import serial
import time
import sys

#This script demonstrates using pyserial to communicate with the hapkit
#run spring_wall_damper_ISR.ino from Arduino IDE and then run this to connect
#You should be able to command which environment is displayed from your terminal

# --- CONFIGURATION ---
# Windows: 'COM3', 'COM4' | Mac/Linux: '/dev/ttyACM0', '/dev/ttyUSB0'
ARDUINO_PORT = "/dev/ttyUSB0" 
BAUD_RATE = 115200

def print_menu():
    print("\n" + "="*35)
    print("     HAPTIC CONTROLLER INTERFACE     ")
    print("="*35)
    print(" [0] Free Space (Zero Encoder)")
    print(" [1] Virtual Spring")
    print(" [2] Virtual Wall (At 2.0 cm)")
    print(" [3] Virtual Damper")
    print(" [Q] Quit Program")
    print("-"*35)

def main():
    print(f"Connecting to haptic device on {ARDUINO_PORT}...")
    
    try:
        # Open the serial port
        ser = serial.Serial(port=ARDUINO_PORT, baudrate=BAUD_RATE, timeout=1)
        time.sleep(2) # Wait for Arduino to finish booting up
        print("Connected successfully!")
    except serial.SerialException:
        print(f"Error: Could not open port {ARDUINO_PORT}. Is the Serial Monitor open?")
        sys.exit(1)

    try:
        while True:
            # 1. Clear out the massive backlog of incoming 'xh' prints
            # This makes sure the program responds instantly to user input
            ser.reset_input_buffer()

            # 2. Show the menu and wait for the student to press Enter
            print_menu()
            user_input = input("Select Environment (0-3 or Q): ").strip().upper()

            # 3. Handle exit condition
            if user_input == 'Q':
                print("\nResetting device to Free Space and exiting...")
                ser.write(b'0') # Send '0' to safely turn off motor forces
                break

            # 4. Handle menu selection
            if user_input in ['0', '1', '2', '3']:
                # Send the single character choice to the Arduino
                ser.write(user_input.encode('utf-8'))
                print(f"\n>> Command sent: Switched to Mode {user_input}")
                
                # 5. Read just the single next line from Arduino to prove it's working
                time.sleep(0.1) # Give the Arduino a tiny moment to respond
                if ser.in_waiting > 0:
                    current_position = ser.readline().decode('utf-8').strip()
                    print(f"Current Handle Position: {current_position} cm")
            else:
                print("\n[Invalid Option] Please enter 0, 1, 2, 3, or Q.")

    except KeyboardInterrupt:
        print("\nProgram interrupted.")
    
    finally:
        # Always close the port when done
        ser.close()
        print("Serial link closed.")

if __name__ == "__main__":
    main()
