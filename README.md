
<h1 align="center">🤝 Pioneer 2 - Interface | Electronic MVP</h1>
<p align="center"> Electronic Module for the connection between pioneer 2 and Esp32</p>

![boards](https://github.com/user-attachments/assets/76028a82-7d8b-4769-86a7-74329bbe0730)

## 🌎 Real board in the robot

![real_robot_with_the_board](https://github.com/user-attachments/assets/055b9b63-8b5c-4baa-b7e7-15dbc2f9af0a)

## 🧠 Overall board functionality

![board_idea](https://github.com/user-attachments/assets/24a77449-1310-4927-a293-3f0e926e67d7)


The Pioneer 2DX Interface has the mission to perform the computational logic behind the communication with the Pioneer 2DX and enable for an outside electronic device to send and receive data from the robot. Being more specific for this project, the Pioneer 2DX interface firmware has three main tasks:

- Connect the ESP32 D1 Mini with the bluepill
- Connect the ESP32 D1 Mini with a PS5 controller
- Connect the ESP32 D1 Mini with the pioneer 2DX

The Pioneer 2DX interface allows either a PS5 controller or the Bluepill device to send data to the ESP32, but never simultaneously, as shown in Figure 21. The complete code developed for this interface is available in the GitHub repository of our organization.

The PS5 Controller was added to the project to make it easier to validate the robot’s mechanical and electrical flow without needing to use the autonomous code. This approach allows for identifying issues without suspecting the bluepill code.

![algorithm logic](https://github.com/user-attachments/assets/78fb4d0a-44da-49eb-ada0-e349c0882f95)