import random
import os
import time

try:
    while True:
        # Generate three random numbers
        rand1 = random.randint(1, 100)
        rand2 = random.randint(1, 100)
        rand3 = random.randint(1, 100)

        # Clear the console
        os.system('cls' if os.name == 'nt' else 'clear')

        # Print the numbers in the required format on separate lines
        print(f"number 1: {rand1}")
        print(f"number 2: {rand2}")
        print(f"number 3: {rand3}")
        
        # Wait for a moment before generating new numbers
        time.sleep(1)  # Adjust the sleep time as desired

except KeyboardInterrupt:
    print("\nProgram stopped by user.")

