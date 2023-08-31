# Quadrature-Encoder-Test-Kit
Quadrature Encoder Test Kit based around STM32F411 Development Board.

The aim of this project was to design a low cost, embedded system capable of testing the working condition of
quadrature encoders. This involved measuring the pulses per revolution, individual channel counts (A, B and Z),
relative position of motion and rpm/frequency of rotation. The system shows these measurements on a compact
screen, and is comprehendible to someone with a trades-persons education level.

# Author
Ryan Lederhose

# Functionality
* Encoder Pulse Count
* Individual Channel Count
* Rotation Direction
* RPM and Frequency Calculation
* Relative Degrees of Rotation Measurement
* 1.8inch TFT Display w/ 25Hz Update Rate
* Screw terminals facilitating strong connection to encoder wires

![pcb1](https://github.com/ryanlederhose/Quadrature-Encoder-Test-Kit/assets/112144274/4bcf3829-eec9-49c2-9544-171fd0a54ef0)

# Build Instructions
1. Clone the GitHub repository
```bash
git clone https://github.com.ryanlederhose/Quadrature-Encoder-Test-Kit.git
```
2. Ensure STM32CubeIDE is downloaded on your personal device
3. Within the CubeIDE, start a new STM32 Project from Existing STM32CubeMX Configuration file (*.ioc)
4. Use the .ioc file from
```bash
./Quadrature-Encoder-Test-Kit/embedded/quadrature-decoder/quadrature-decoder.ioc
```
5. Replace the Core folder from the project with the Core folder form:
```bash
./Quadrature-Encoder-Test-Kit/embedded/quadrature-decoder/Core/
```

6. Connect your debugger (e.g. STLink) as such

![debug-pinout](https://github.com/ryanlederhose/Quadrature-Encoder-Test-Kit/assets/112144274/9d5e1365-80a3-438a-8b87-3ebe7ba733b1)

7. Run the debugger software on STM32CubeIDE

# Use Instructions
1. Use a 12V, 1A DC Power Supply (2.1mm plug). It's important not to supply more than the recommended voltage as it will fry the voltage regultator
   
![power-jack](https://github.com/ryanlederhose/Quadrature-Encoder-Test-Kit/assets/112144274/5afe03cb-1b9a-457f-8386-ce00334d9a43)

2. Connect the encoder to the 8-pin plug as such:

![labelle-pinout](https://github.com/ryanlederhose/Quadrature-Encoder-Test-Kit/assets/112144274/5c5dbc79-605e-4109-a90a-1e3541169207)

Typically, encoders will follow the same general colour scheme. In practice, it is not essential to connect the complementary channels (i.e. AN, BN and ZN) as shown as they
are all internally connected to ground. As long as they are connected to the ground pins, the device will function properly.

3. Connect the 8-pin plug to the socket connection on the PCB

![pcb-connection](https://github.com/ryanlederhose/Quadrature-Encoder-Test-Kit/assets/112144274/b6437086-1fee-4c5a-94a7-ed9bcbe4a2c6)

4. Upon rotating the shaft of the encoder, the LCD display will now update with the appropriate measurements.

![lcd3](https://github.com/ryanlederhose/Quadrature-Encoder-Test-Kit/assets/112144274/530cb3f3-f087-41fb-b29e-648ba638a5db)

5. It is a requirement for correct measurements that the encoder completes at least one full revolution in the direction of rotation starting from the origin point
(i.e. origin meaning when the Z channel count ticks over). This point is set by the encoder itself and not the system.
