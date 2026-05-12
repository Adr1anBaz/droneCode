"""
Drone Orientation Control - Interactive Menu v2
Safety: Max 50% PWM, slow ramp, pre-spin phase

Connects via WebSocket to ESP32 AP (DroneControl / drone1234)
WebSocket: ws://192.168.4.1:81
"""

import asyncio
import json
import sys
import os
import time

try:
    import websockets
except ImportError:
    print("Instalando websockets...")
    os.system(f"{sys.executable} -m pip install websockets")
    import websockets

WS_URI = "ws://192.168.4.1:81"

latest_data = {}
connected = False
log_file = None


def clear():
    os.system('cls' if os.name == 'nt' else 'clear')


def open_log():
    global log_file
    filename = f"drone_log_{time.strftime('%Y%m%d_%H%M%S')}.csv"
    log_file = open(filename, 'w')
    log_file.write("timestamp,state,roll,pitch,yaw,target_roll,target_pitch,target_yaw,pwm_t1,pwm_t2,pwm_t3,pwm_t4,pwm_a1,pwm_a2,pwm_a3,pwm_a4\n")
    print(f"  Log guardado en: {filename}")
    return filename


def log_data(data):
    if log_file and "roll" in data:
        t = time.time()
        target = data.get("target", [0, 0, 0])
        pwm_t = data.get("pwm_target", [0, 0, 0, 0])
        pwm_a = data.get("pwm_actual", [0, 0, 0, 0])
        state = data.get("state", "?")
        log_file.write(f"{t:.3f},{state},{data['roll']:.2f},{data['pitch']:.2f},{data['yaw']:.2f},")
        log_file.write(f"{target[0]:.1f},{target[1]:.1f},{target[2]:.1f},")
        log_file.write(f"{pwm_t[0]:.1f},{pwm_t[1]:.1f},{pwm_t[2]:.1f},{pwm_t[3]:.1f},")
        log_file.write(f"{pwm_a[0]:.1f},{pwm_a[1]:.1f},{pwm_a[2]:.1f},{pwm_a[3]:.1f}\n")
        log_file.flush()


def print_header():
    print("=" * 60)
    print("    DRONE ORIENTATION CONTROL v2 - GyroTuner")
    print("    Max PWM: 50% | Ramp: 0.5%/ciclo | Pre-spin: 3s")
    print("=" * 60)


def print_telemetry():
    if not latest_data:
        print("  [Sin datos de telemetria]")
        return

    if "cal_progress" in latest_data:
        prog = latest_data["cal_progress"]
        bar = "#" * (prog // 5) + "." * (20 - prog // 5)
        print(f"  Calibrando: [{bar}] {prog}%")
        return

    roll = latest_data.get("roll", 0)
    pitch = latest_data.get("pitch", 0)
    yaw = latest_data.get("yaw", 0)
    target = latest_data.get("target", [0, 0, 0])
    pwm_t = latest_data.get("pwm_target", [0, 0, 0, 0])
    pwm_a = latest_data.get("pwm_actual", [0, 0, 0, 0])
    state = latest_data.get("state", "?")

    print(f"  Estado: {state}")
    print(f"  +-------------------------------------------+")
    print(f"  |  Angulos actuales (grados):               |")
    print(f"  |    Roll:  {roll:+7.2f}  (target: {target[0]:+.1f})   |")
    print(f"  |    Pitch: {pitch:+7.2f}  (target: {target[1]:+.1f})   |")
    print(f"  |    Yaw:   {yaw:+7.2f}  (target: {target[2]:+.1f})   |")
    print(f"  +-------------------------------------------+")
    print(f"  |  PWM Target -> Actual (%):                |")
    print(f"  |    M1(FL): {pwm_t[0]:5.1f} -> {pwm_a[0]:5.1f}           |")
    print(f"  |    M2(FR): {pwm_t[1]:5.1f} -> {pwm_a[1]:5.1f}           |")
    print(f"  |    M3(BL): {pwm_t[2]:5.1f} -> {pwm_a[2]:5.1f}           |")
    print(f"  |    M4(BR): {pwm_t[3]:5.1f} -> {pwm_a[3]:5.1f}           |")
    print(f"  +-------------------------------------------+")


def print_menu():
    print()
    print("  [1] Calibrar IMU")
    print("  [2] Iniciar control (pre-spin + PID)")
    print("  [3] Cambiar angulo objetivo")
    print("  [4] Cambiar throttle base")
    print("  [5] Ajustar PID")
    print("  [6] Detener motores")
    print("  [7] Monitor en vivo")
    print("  [8] TEST MANUAL - todos a 5% (verificar motores)")
    print("  [0] Salir")
    print()


async def receive_loop(ws):
    global latest_data
    try:
        async for msg in ws:
            data = json.loads(msg)
            if "event" in data:
                if data["event"] == "calibrated":
                    print("\n  OK Calibracion completada!")
                    print(f"    Offsets gyro: gx={data['offsets']['gx']:.4f}, gy={data['offsets']['gy']:.4f}, gz={data['offsets']['gz']:.4f}")
                elif data["event"] == "pre_spin":
                    print("\n  >> Pre-spin: subiendo todos a 5% lentamente (3s)...")
                elif data["event"] == "running":
                    print("\n  OK Control PID activo - manteniendo posicion")
                elif data["event"] == "stopped":
                    print("\n  OK Motores detenidos")
                elif data["event"] == "manual_test":
                    print("\n  >> TEST: Todos los motores subiendo a 5%...")
                    print("     Verifica que los 4 giren.")
                elif data["event"] == "test_stopped":
                    print("\n  OK Test terminado, motores apagados")
                elif data["event"] == "target_set":
                    print(f"\n  OK Nuevo target: roll={data['roll']}d, pitch={data['pitch']}d, yaw={data['yaw']}d")
                elif data["event"] == "throttle_set":
                    print(f"\n  OK Throttle base: {data['value']}%")
                elif data["event"] == "pid_set":
                    print(f"\n  OK PID {data['axis']} actualizado")
                elif "error" in data:
                    print(f"\n  ERROR: {data['error']}")
            else:
                latest_data = data
                log_data(data)
    except websockets.exceptions.ConnectionClosed:
        pass


async def calibrate(ws):
    clear()
    print_header()
    print()
    print("  +=============================================+")
    print("  |         CALIBRACION DE LA IMU               |")
    print("  +=============================================+")
    print("  |  Instrucciones:                             |")
    print("  |                                             |")
    print("  |  1. Coloca el dron en el gimbal             |")
    print("  |  2. Ponlo lo mas DERECHO posible            |")
    print("  |     (nivelado, sin inclinacion)             |")
    print("  |  3. NO LO TOQUES durante la calibracion    |")
    print("  |  4. Espera ~3 segundos                      |")
    print("  |                                             |")
    print("  |  La posicion actual sera tu (0d,0d,0d)     |")
    print("  +=============================================+")
    print()
    input("  Presiona ENTER cuando este listo y quieto...")
    print()
    print("  Calibrando... NO muevas el dron!")
    await ws.send("calibrate")
    await asyncio.sleep(4)
    print()
    input("  Presiona ENTER para volver al menu...")


async def start_control(ws):
    print()
    print("  Secuencia de inicio:")
    print("    1. Pre-spin: todos los motores suben a 5% (3 segundos)")
    print("    2. PID se activa y mantiene la posicion calibrada")
    print()
    print("  El PWM sube LENTO (max 0.5% por ciclo)")
    print("  Maximo absoluto: 50%")
    print()
    conf = input("  Continuar? (s/n): ").strip().lower()
    if conf != 's':
        print("  Cancelado.")
        return
    await ws.send("start")
    await asyncio.sleep(0.5)


async def set_target(ws):
    print()
    print("  Ingresa los angulos deseados en grados:")
    print("  (El dron rotara LENTAMENTE hacia esos angulos)")
    try:
        roll = float(input("    Roll  (izq/der) [grados]: "))
        pitch = float(input("    Pitch (frente/atras) [grados]: "))
        yaw = float(input("    Yaw   (rotacion) [grados]: "))
        await ws.send(f"target:{roll},{pitch},{yaw}")
    except ValueError:
        print("  ERROR: Valores invalidos")
    await asyncio.sleep(0.3)


async def set_throttle(ws):
    print()
    print(f"  Throttle es la potencia BASE de todos los motores.")
    print(f"  El PID suma/resta sobre este valor.")
    print(f"  Rango permitido: 5% - 50%")
    try:
        val = float(input("  Nuevo throttle base [%]: "))
        if val < 5 or val > 50:
            print("  ERROR: Fuera de rango (5-50)")
            return
        await ws.send(f"throttle:{val}")
    except ValueError:
        print("  ERROR: Valor invalido")
    await asyncio.sleep(0.3)


async def set_pid(ws):
    print()
    print("  Ajustar PID - selecciona eje:")
    print("    [1] Roll   (Kp=1.0, Ki=0.01, Kd=0.5)")
    print("    [2] Pitch  (Kp=1.0, Ki=0.01, Kd=0.5)")
    print("    [3] Yaw    (Kp=1.5, Ki=0.02, Kd=0.7)")
    print()
    print("  Consejo: empieza con Kp bajo (0.5), Ki=0, Kd=0")
    print("  Sube Kp hasta que oscile, luego agrega Kd para frenar")
    axis_map = {"1": "roll", "2": "pitch", "3": "yaw"}
    choice = input("  Eje: ")
    if choice not in axis_map:
        print("  ERROR: Opcion invalida")
        return
    axis = axis_map[choice]
    try:
        kp = float(input(f"    Kp para {axis}: "))
        ki = float(input(f"    Ki para {axis}: "))
        kd = float(input(f"    Kd para {axis}: "))
        await ws.send(f"pid:{axis},{kp},{ki},{kd}")
    except ValueError:
        print("  ERROR: Valores invalidos")
    await asyncio.sleep(0.3)


async def stop_motors(ws):
    await ws.send("stop")
    await asyncio.sleep(0.3)


async def manual_test(ws):
    print()
    print("  +=============================================+")
    print("  |         TEST MANUAL DE MOTORES              |")
    print("  +=============================================+")
    print("  |  Todos los motores iran a 5% PWM           |")
    print("  |  Verifica que los 4 giren correctamente    |")
    print("  |                                             |")
    print("  |  M1(FL): Negro/Blanco - debe girar CCW     |")
    print("  |  M2(FR): Rojo/Azul    - debe girar CW      |")
    print("  |  M3(BL): Rojo/Azul    - debe girar CW      |")
    print("  |  M4(BR): Negro/Blanco - debe girar CCW     |")
    print("  +=============================================+")
    print()
    conf = input("  Encender motores a 5%? (s/n): ").strip().lower()
    if conf != 's':
        print("  Cancelado.")
        return

    await ws.send("test")
    print()
    print("  Motores encendidos. Observa que los 4 giren.")
    print()
    input("  Presiona ENTER para APAGAR y volver al menu...")
    await ws.send("test_stop")
    await asyncio.sleep(0.5)


async def live_monitor(ws):
    clear()
    print("  MONITOR EN VIVO (Ctrl+C para salir)")
    print("  " + "-" * 50)
    print()
    try:
        while True:
            await asyncio.sleep(0.15)
            clear()
            print("  MONITOR EN VIVO (Ctrl+C para salir)")
            print("  " + "-" * 50)
            print_telemetry()
    except KeyboardInterrupt:
        pass


async def main():
    global connected

    clear()
    print_header()
    print()
    print("  Conectando a DroneControl (192.168.4.1:81)...")
    print("  Asegurate de estar conectado al WiFi 'DroneControl'")
    print("  Password: drone1234")
    print()

    log_filename = open_log()
    print()

    try:
        async with websockets.connect(WS_URI, ping_interval=20) as ws_conn:
            connected = True
            print("  OK Conectado al dron!")
            print()

            recv_task = asyncio.create_task(receive_loop(ws_conn))

            while True:
                clear()
                print_header()
                print_telemetry()
                print_menu()

                choice = input("  Opcion: ").strip()

                if choice == "1":
                    await calibrate(ws_conn)
                elif choice == "2":
                    await start_control(ws_conn)
                elif choice == "3":
                    await set_target(ws_conn)
                elif choice == "4":
                    await set_throttle(ws_conn)
                elif choice == "5":
                    await set_pid(ws_conn)
                elif choice == "6":
                    await stop_motors(ws_conn)
                elif choice == "7":
                    await live_monitor(ws_conn)
                elif choice == "8":
                    await manual_test(ws_conn)
                elif choice == "0":
                    await stop_motors(ws_conn)
                    print("  Saliendo...")
                    recv_task.cancel()
                    break
                else:
                    print("  Opcion no valida")
                    await asyncio.sleep(1)

                await asyncio.sleep(0.2)

    except ConnectionRefusedError:
        print("  ERROR: No se pudo conectar. Verifica:")
        print("    - Estas conectado al WiFi 'DroneControl'")
        print("    - El ESP32 esta encendido")
    except Exception as e:
        print(f"  ERROR: {e}")
    finally:
        if log_file:
            log_file.close()
            print(f"  Log guardado: {log_filename}")


if __name__ == "__main__":
    asyncio.run(main())
