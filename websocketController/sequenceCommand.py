import websocket
import random
import time


# Function to send a command via an existing WebSocket connection
def send_command(ws, command):
    print(f"Sending command: {command}")
    ws.send(command)


# Function to set the loop_scale value
def set_loop_scale(ws, loop_scale):
    command = f"set_loop_scale:{loop_scale}"  # Format for setting loop_scale
    print(f"Setting loop_scale to {loop_scale}...")
    send_command(ws, command)
    response = ws.recv()
    print(f"Response: {response}")

# Function to check the current status
def check_status(ws):
    command = "get_status"  # Command to request status
    print("Requesting status...")
    send_command(ws, command)
    response = ws.recv()
    print(f"Current status: {response}")


# Function to send the initial sequence of commands
def send_initial_sequence(ws, initial_commands):
    for command in initial_commands:
        send_time = time.time()
        send_command(ws, command)
        # Wait for the "completed" message before sending the next command
        # while True:
        #     response = ws.recv()
        #     print(f"Received from server: {response}")
        #     print("execution time= ", time.time()-send_time)
        #     if response == "completed":
        #         print(f"Command {command} completed.")
        #         time.sleep(0.5)
        #         break
        time.sleep(4)

# Function to send random commands from the available pool
def send_random_commands(ws, commands):
    while True:
        # Select and send a random command
        random_command = random.choice(commands)
        send_command(ws, random_command)

        # Wait for the "completed" message from the server
        while True:
            response = ws.recv()
            print(f"Received from server: {response}")
            if response == "completed":
                print(f"Command {random_command} completed, sending next random command...")
                time.sleep(0.5)
                break


# Main function to manage the WebSocket connection and send random commands conditionally
def main():
    # Array of possible command sequences
    random_commands = ['f08r02f05', 'b09l02f06', 'r03f05', 'l02b05', 'f05r02', 's05','f12r03','b12l03']  # Add more commands as needed
    # initial_commands = ["f10s02f10r05s02F20","b11s02b11l05f08r06F20"]  # fast sequence
    initial_commands = ["f10s02f10r02s02f22","b11s02b11l05f08r06f20"]  # slow sequence
    # initial_commands = ["l05f10r05f10s02f20r05s02f22"]  # left wing
    # initial_commands = ["f22s02","b22s02"]  # slow sequence

    # Connect to the WebSocket server on the bot
    bot_ip = "ws://192.168.57.196:81"  # Replace with your bot's IP address
    # jasira Ip: 192.168.104.196:81
    # hashar: 192.168.78.196:81
    ws = websocket.WebSocket()
    ws.connect(bot_ip)
    print("Connected to WebSocket server")
    # check_status(ws)
    set_loop_scale(ws,10)
    time.sleep(0.5)
    # check_status(ws)

    # time.sleep(100)


    try:
        print("Starting initial sequence...")
        send_initial_sequence(ws, initial_commands)
        time.sleep(3)

        # After the initial sequence, send random commands
        print("Switching to random command sequence...")
        send_random_commands(ws, random_commands)

    except KeyboardInterrupt:
        print("Terminating command loop...")

    finally:
        # Close the WebSocket connection when done
        ws.close()
        set_loop_scale(ws, 5)
        print("WebSocket connection closed")


    # initial_command = "f20r07f22b20l05b02"   # mad
    # # initial_command = "f10s02f10r05s02f16b20l05f05"  # less mad
    # send_command(ws, initial_command)
    # time.sleep(3)
    # second_command = "l10s02f08r15s02R10f10L10"
    # time.sleep(4)
    #
    # try:
    #     # Continuously send commands based on server response
    #     while True:
    #         # Select and send a random command
    #         random_command = random.choice(commands)
    #         send_command(ws, random_command)
    #
    #         # Wait for the "completed" message from the server
    #         while True:
    #             response = ws.recv()
    #             print(f"Received from server: {response}")
    #             if response == "completed":
    #                 print("Server confirmed completion, sending next command...")
    #                 time.sleep(0.5)
    #                 break  # Exit the inner loop to send the next command
    #
    # except KeyboardInterrupt:
    #     print("Terminating command loop...")
    #
    # finally:
    #     # Close the WebSocket connection when done
    #     ws.close()
    #     print("WebSocket connection closed")


if __name__ == "__main__":
    main()
