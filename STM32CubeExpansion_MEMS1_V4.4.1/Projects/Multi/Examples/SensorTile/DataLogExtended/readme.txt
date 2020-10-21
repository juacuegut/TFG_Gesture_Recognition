/**
  @page Sensor DataLog Demo for Unicleo GUI Application based on SensorTile board

  @verbatim
  ******************** (C) COPYRIGHT 2018 STMicroelectronics *******************
  * @file    readme.txt
  * @version V1.0.0
  * @date    26-February-2018
  * @brief   This application contains an example which shows how to obtain data
  *          from various sensors on sensor expansion board.
  *          The communication is done using a UART connection with PC and the Unicleo Utility.
  ******************************************************************************
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  @endverbatim

@par Example Description

Main function is to show how to use SensorTile to send data using UART
to a connected PC or Desktop and display it on specific application Unicleo,
which is developed by STMicroelectronics and provided separately, not in binary with this package.
After connection has been established:
- the user can view the data from various on-board environment sensors like Temperature,
Humidity, and Pressure.
- the user can also view data from various on-board MEMS sensors as well like Accelerometer,
Gyroscope, and Magnetometer.
- the user can also visualize this data as graphs using Unicleo application.


@par Hardware and Software environment

  - This example runs on SensorTile device.
  - This example has been tested with SesorTile.


@par How to use it ?

This package contains projects for 3 IDEs viz. IAR, µVision and System Workbench. In order to make the
program work, you must do the following:
 - WARNING: before opening the project with any toolchain be sure your folder
   installation path is not too in-depth since the toolchain may report errors
   after building.
 - WARNING: this sample application is only compatible with the Unicleo GUI.

For IAR:
 - Open IAR toolchain (this firmware has been successfully tested with
   Embedded Workbench V7.80.4).
 - Open the IAR project file EWARM\STM32L476RG-SensorTile\Project.eww.
 - Rebuild all files and load your image into target memory.
 - Run the example.

For µVision:
 - Open µVision 5 toolchain (this firmware has been
   successfully tested with MDK-ARM Professional Version: 5.22).
 - Open the µVision project file MDK-ARM\STM32L476RG-SensorTile\Project.uvprojx.
 - Rebuild all files and load your image into target memory.
 - Run the example.

For System Workbench:
 - Open System Workbench for STM32 (this firmware has been
   successfully tested with System Workbench for STM32 Version 1.14.0.20170306).
 - Set the default workspace proposed by the IDE (please be sure that there are not spaces in the workspace path).
 - Press "File" -> "Import" -> "Existing Projects into Workspace"; press "Browse" in the "Select root directory" and choose the path where the System
   Workbench project is located (it should be SW4STM32\STM32L476RG-SensorTile\SensorTile-DataLogExtended).
 - Rebuild all files and load your image into target memory.
 - Run the example.


 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
