"""
Drone PCB Test Controller - Python
Connect your computer to WiFi network "DroneTest" (password: drone1234)
Then run this script to control motors via PWM percentage.
"""

import requests
import time
import sys

DRONE_IP = "192.168.4.1"
BASE_URL = f"http://{DRONE_IP}"


def set_motor(motor: int, pwm: float):
    """Set a single motor PWM (1-4, 0-100%)."""
    r = requests.post(f"{BASE_URL}/motor", json={"motor": motor, "pwm": pwm})
    print(f"  Motor {motor} -> {pwm}%  |  Response: {r.json()}")
    return r.json()


def set_all(pwm: float):
    """Set all motors to same PWM (0-100%)."""
    r = requests.post(f"{BASE_URL}/all", json={"pwm": pwm})
    print(f"  All motors -> {pwm}%  |  Response: {r.json()}")
    return r.json()


def stop():
    """Stop all motors immediately."""
    r = requests.post(f"{BASE_URL}/stop")
    print(f"  STOP  |  Response: {r.json()}")
    return r.json()


def status():
    """Get current motor PWM values."""
    r = requests.get(f"{BASE_URL}/status")
    data = r.json()
    motors = data["motors"]
    print(f"  Status: M1={motors[0]}% M2={motors[1]}% M3={motors[2]}% M4={motors[3]}%")
    return data


def test_individual_motors():
    """Test each motor individually at low power."""
    print("\n--- Individual Motor Test (20% PWM, 2s each) ---")
    print("This tests each motor one by one.")
    input("Press Enter to begin (make sure propellers are OFF)...")

    for m in range(1, 5):
        print(f"\n  Testing Motor {m}...")
        set_motor(m, 20)
        time.sleep(2)
        set_motor(m, 0)
        time.sleep(0.5)

    print("\n  Individual test complete.")


def test_all_motors():
    """Test all motors at increasing power levels."""
    print("\n--- All Motors Ramp Test ---")
    print("This ramps all motors from 10% to 50% in steps.")
    input("Press Enter to begin (make sure propellers are OFF)...")

    for pwm in [10, 20, 30, 40, 50]:
        print(f"\n  Setting all to {pwm}%...")
        set_all(pwm)
        time.sleep(2)

    print("\n  Stopping all motors...")
    stop()
    print("  Ramp test complete.")


def interactive_mode():
    """Interactive control via terminal."""
    print("\n--- Interactive Mode ---")
    print("Commands:")
    print("  m <motor> <pwm>  - Set motor (1-4) to PWM (0-100%)")
    print("  a <pwm>          - Set all motors to PWM (0-100%)")
    print("  s                - Stop all motors")
    print("  t                - Show status")
    print("  q                - Quit (stops motors first)")
    print()

    while True:
        try:
            cmd = input("> ").strip().split()
            if not cmd:
                continue

            if cmd[0] == 'q':
                stop()
                break
            elif cmd[0] == 's':
                stop()
            elif cmd[0] == 't':
                status()
            elif cmd[0] == 'm' and len(cmd) == 3:
                set_motor(int(cmd[1]), float(cmd[2]))
            elif cmd[0] == 'a' and len(cmd) == 2:
                set_all(float(cmd[1]))
            else:
                print("  Unknown command. Try: m 1 50 | a 30 | s | t | q")

        except KeyboardInterrupt:
            print("\n  Emergency stop!")
            stop()
            break
        except requests.ConnectionError:
            print("  Connection error - is the drone on and are you connected to DroneTest?")
        except Exception as e:
            print(f"  Error: {e}")


def main():
    print("=" * 50)
    print("  DRONE PCB TEST CONTROLLER")
    print("=" * 50)
    print(f"\n  Target: {BASE_URL}")
    print("  Make sure you are connected to WiFi: DroneTest")
    print()

    # Check connection
    try:
        r = requests.get(f"{BASE_URL}/status", timeout=3)
        print("  Connected to drone!")
        status()
    except requests.ConnectionError:
        print("  ERROR: Cannot connect to drone.")
        print("  1. Make sure the ESP32 is powered on")
        print("  2. Connect to WiFi network 'DroneTest' (password: drone1234)")
        print("  3. Try again")
        sys.exit(1)

    print("\n  Select test mode:")
    print("  1. Individual motor test (one by one)")
    print("  2. All motors ramp test")
    print("  3. Interactive mode (manual control)")
    print()

    choice = input("  Choice (1/2/3): ").strip()

    if choice == '1':
        test_individual_motors()
    elif choice == '2':
        test_all_motors()
    elif choice == '3':
        interactive_mode()
    else:
        print("  Invalid choice, entering interactive mode...")
        interactive_mode()


if __name__ == "__main__":
    main()
