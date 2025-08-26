import time
from mini_bdx_runtime.feet_contacts import FeetContacts

def main():
    feet = FeetContacts()
    try:
        print("Press Ctrl+C to stop.")
        while True:
            left, right = feet.get()
            print(f"Left foot: {'contact' if left else 'no contact'}, Right foot: {'contact' if right else 'no contact'}")
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nExiting feet test.")
    finally:
        feet.stop()

if __name__ == "__main__":
    main()