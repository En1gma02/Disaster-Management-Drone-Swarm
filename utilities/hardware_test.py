import time
from pymavlink import mavutil

SYSTEM_ADDR = "serial:///dev/ttyUSB0:57600" # Typical radio telemetry port
SERVO_INSTANCE = 9

def play_siren(master):
    tune_str = "T255 O7 L8 c e c e c e P8 c e c e c e P8 c e c e"
    print(f"   >>> 🔊 ACTIVATING BUZZER (Siren) <<<")
    try:
        if hasattr(master.mav, 'play_tune_v2_send'):
            master.mav.play_tune_v2_send(master.target_system, master.target_component, 1, tune_str.encode('ascii'))
    except Exception as e:
        print(f"Buzzer Error: {e}")

def set_servo(master, angle):
    # Map 0-180 to 500-2500
    pwm = int(500 + (angle / 180.0) * 2000)
    print(f"   >>> ⚙️ MOVING SERVO {SERVO_INSTANCE} to {angle} deg (PWM {pwm}) <<<")
    master.mav.command_long_send(master.target_system, master.target_component, 183, 0, SERVO_INSTANCE, pwm, 0, 0, 0, 0, 0)

def main():
    print("="*50)
    print("HARDWARE DIAGNOSTIC TEST")
    print("="*50)
    
    try:
        master = mavutil.mavlink_connection(SYSTEM_ADDR, baud=57600)
        master.wait_heartbeat(timeout=10)
        print(f"✅ Connected to Flight Controller (SysID: {master.target_system})")
    except Exception as e:
        print(f"❌ Connection Failed: {e}")
        return

    while True:
        print("\nOptions:")
        print("1. Test Buzzer (Siren)")
        print("2. Test Servo (0 degrees)")
        print("3. Test Servo (90 degrees)")
        print("4. Test Servo (180 degrees)")
        print("5. Exit")
        
        choice = input("Select an option: ")
        if choice == '1':
            play_siren(master)
        elif choice == '2':
            set_servo(master, 0)
        elif choice == '3':
            set_servo(master, 90)
        elif choice == '4':
            set_servo(master, 180)
        elif choice == '5':
            break
        else:
            print("Invalid choice.")
        time.sleep(1)

if __name__ == "__main__":
    main()
