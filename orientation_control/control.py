"""
Drone Orientation Control - Interactive Menu
Connects via WebSocket to ESP32 AP (DroneControl / drone1234)
WebSocket: ws://192.168.4.1:81
"""

import asyncio
import json
import sys
import os

try:
    import websockets
except ImportError:
    print("Instalando websockets...")
    os.system(f"{sys.executable} -m pip install websockets")
    import websockets

WS_URI = "ws://192.168.4.1:81"

latest_data = {}
connected = False


def clear():
    os.system('cls' if os.name == 'nt' else 'clear')


def print_header():
    print("=" * 60)
    print("       DRONE ORIENTATION CONTROL - GyroTuner")
    print("=" * 60)


def print_telemetry():
    if not latest_data:
        print("  [Sin datos de telemetría]")
        return

    if "cal_progress" in latest_data:
        prog = latest_data["cal_progress"]
        bar = "█" * (prog // 5) + "░" * (20 - prog // 5)
        print(f"  Calibrando: [{bar}] {prog}%")
        return

    roll = latest_data.get("roll", 0)
    pitch = latest_data.get("pitch", 0)
    yaw = latest_data.get("yaw", 0)
    target = latest_data.get("target", [0, 0, 0])
    pwm = latest_data.get("pwm", [0, 0, 0, 0])
    state = latest_data.get("state", "?")

    print(f"  Estado: {state}")
    print(f"  ┌─────────────────────────────────────────┐")
    print(f"  │  Ángulos actuales (°):                   │")
    print(f"  │    Roll:  {roll:+7.2f}   (target: {target[0]:+.1f})  │")
    print(f"  │    Pitch: {pitch:+7.2f}   (target: {target[1]:+.1f})  │")
    print(f"  │    Yaw:   {yaw:+7.2f}   (target: {target[2]:+.1f})  │")
    print(f"  ├─────────────────────────────────────────┤")
    print(f"  │  PWM motores (%):                        │")
    print(f"  │    M1(FL): {pwm[0]:5.1f}   M2(FR): {pwm[1]:5.1f}     │")
    print(f"  │    M3(BL): {pwm[2]:5.1f}   M4(BR): {pwm[3]:5.1f}     │")
    print(f"  └─────────────────────────────────────────┘")


def print_menu():
    print()
    print("  [1] Calibrar IMU")
    print("  [2] Iniciar control (mantener posición calibrada)")
    print("  [3] Cambiar ángulo objetivo")
    print("  [4] Cambiar throttle base")
    print("  [5] Ajustar PID")
    print("  [6] Detener motores")
    print("  [7] Monitor en vivo")
    print("  [0] Salir")
    print()


async def receive_loop(ws):
    global latest_data
    try:
        async for msg in ws:
            data = json.loads(msg)
            if "event" in data:
                if data["event"] == "calibrated":
                    print("\n  ✓ Calibración completada!")
                    print(f"    Offsets gyro: gx={data['offsets']['gx']:.4f}, gy={data['offsets']['gy']:.4f}, gz={data['offsets']['gz']:.4f}")
                elif data["event"] == "running":
                    print("\n  ✓ Control PID activo - manteniendo posición")
                elif data["event"] == "stopped":
                    print("\n  ✓ Motores detenidos")
                elif data["event"] == "target_set":
                    print(f"\n  ✓ Nuevo target: roll={data['roll']}°, pitch={data['pitch']}°, yaw={data['yaw']}°")
                elif data["event"] == "throttle_set":
                    print(f"\n  ✓ Throttle base: {data['value']}%")
                elif data["event"] == "pid_set":
                    print(f"\n  ✓ PID {data['axis']} actualizado")
                elif "error" in data:
                    print(f"\n  ✗ Error: {data['error']}")
            else:
                latest_data = data
    except websockets.exceptions.ConnectionClosed:
        pass


async def calibrate(ws):
    clear()
    print_header()
    print()
    print("  ╔═══════════════════════════════════════════╗")
    print("  ║         CALIBRACIÓN DE LA IMU             ║")
    print("  ╠═══════════════════════════════════════════╣")
    print("  ║  Instrucciones:                           ║")
    print("  ║                                           ║")
    print("  ║  1. Coloca el dron en el gimbal           ║")
    print("  ║  2. Ponlo lo más DERECHO posible          ║")
    print("  ║     (nivelado, sin inclinación)           ║")
    print("  ║  3. NO LO TOQUES durante la calibración  ║")
    print("  ║  4. Espera ~3 segundos                    ║")
    print("  ║                                           ║")
    print("  ║  La posición actual será tu (0°,0°,0°)   ║")
    print("  ╚═══════════════════════════════════════════╝")
    print()
    input("  Presiona ENTER cuando esté listo y quieto...")
    print()
    print("  Calibrando... NO muevas el dron!")
    await ws.send("calibrate")
    await asyncio.sleep(4)
    print("  Listo. La posición actual es el punto cero.")
    print()
    input("  Presiona ENTER para volver al menú...")


async def start_control(ws):
    print()
    print("  Iniciando control PID...")
    print("  El dron mantendrá la posición calibrada (0°,0°,0°)")
    await ws.send("start")
    await asyncio.sleep(0.5)


async def set_target(ws):
    print()
    print("  Ingresa los ángulos deseados en grados:")
    try:
        roll = float(input("    Roll  (izq/der) [°]: "))
        pitch = float(input("    Pitch (frente/atrás) [°]: "))
        yaw = float(input("    Yaw   (rotación) [°]: "))
        await ws.send(f"target:{roll},{pitch},{yaw}")
    except ValueError:
        print("  ✗ Valores inválidos")
    await asyncio.sleep(0.3)


async def set_throttle(ws):
    print()
    print(f"  Throttle actual: ~15%")
    print(f"  Rango permitido: 5% - 80%")
    try:
        val = float(input("  Nuevo throttle base [%]: "))
        if val < 5 or val > 80:
            print("  ✗ Fuera de rango (5-80)")
            return
        await ws.send(f"throttle:{val}")
    except ValueError:
        print("  ✗ Valor inválido")
    await asyncio.sleep(0.3)


async def set_pid(ws):
    print()
    print("  Ajustar PID - selecciona eje:")
    print("    [1] Roll   (actual: Kp=1.5, Ki=0.02, Kd=0.8)")
    print("    [2] Pitch  (actual: Kp=1.5, Ki=0.02, Kd=0.8)")
    print("    [3] Yaw    (actual: Kp=2.0, Ki=0.05, Kd=1.0)")
    axis_map = {"1": "roll", "2": "pitch", "3": "yaw"}
    choice = input("  Eje: ")
    if choice not in axis_map:
        print("  ✗ Opción inválida")
        return
    axis = axis_map[choice]
    try:
        kp = float(input(f"    Kp para {axis}: "))
        ki = float(input(f"    Ki para {axis}: "))
        kd = float(input(f"    Kd para {axis}: "))
        await ws.send(f"pid:{axis},{kp},{ki},{kd}")
    except ValueError:
        print("  ✗ Valores inválidos")
    await asyncio.sleep(0.3)


async def stop_motors(ws):
    await ws.send("stop")
    await asyncio.sleep(0.3)


async def live_monitor(ws):
    clear()
    print("  MONITOR EN VIVO (Ctrl+C para salir)")
    print("  " + "-" * 50)
    try:
        while True:
            await asyncio.sleep(0.1)
            sys.stdout.write("\033[3;0H")  # move cursor
            print_telemetry()
            sys.stdout.flush()
    except KeyboardInterrupt:
        pass


async def main():
    global connected

    clear()
    print_header()
    print()
    print("  Conectando a DroneControl (192.168.4.1:81)...")
    print("  Asegúrate de estar conectado al WiFi 'DroneControl'")
    print("  Password: drone1234")
    print()

    try:
        async with websockets.connect(WS_URI, ping_interval=20) as ws:
            connected = True
            print("  ✓ Conectado al dron!")
            print()

            recv_task = asyncio.create_task(receive_loop(ws))

            while True:
                print_header()
                print_telemetry()
                print_menu()

                choice = input("  Opción: ").strip()

                if choice == "1":
                    await calibrate(ws)
                elif choice == "2":
                    await start_control(ws)
                elif choice == "3":
                    await set_target(ws)
                elif choice == "4":
                    await set_throttle(ws)
                elif choice == "5":
                    await set_pid(ws)
                elif choice == "6":
                    await stop_motors(ws)
                elif choice == "7":
                    await live_monitor(ws)
                elif choice == "0":
                    await stop_motors(ws)
                    print("  Saliendo...")
                    recv_task.cancel()
                    break
                else:
                    print("  Opción no válida")

                await asyncio.sleep(0.2)

    except ConnectionRefusedError:
        print("  ✗ No se pudo conectar. Verifica:")
        print("    - Estás conectado al WiFi 'DroneControl'")
        print("    - El ESP32 está encendido")
    except Exception as e:
        print(f"  ✗ Error: {e}")


if __name__ == "__main__":
    asyncio.run(main())
