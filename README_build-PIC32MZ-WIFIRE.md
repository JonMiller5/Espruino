# Getting Started developing Espruino for PIC32 MCUs using the chipKIT Wi-Fire

NOTE: We wrote these instructions using OS X, but you should be able to follow a similar procedure using Linux or Cygwin under Windows.

Install MPLAB X, MPLAB XC32 v1.42, MPLAB Harmony v2.01b.
Install MPLAB Harmony Configurator (MHC) for Harmony v2.01b, which is a plugin to MPLAB X.

-----------------------------------------------------------

1. Fork the GitHub repository located at https://github.com/jasonkajita/Espruino

TERMINAL:
1. Clone your forked repository to your local machine.
```
$ git clone https://github.com/<username>/Espruino.git
"Cloning into 'Espruino'...
remote: Counting objects: 28775, done.
remote: Compressing objects: 100% (35/35), done.
remote: Total 28775 (delta 8), reused 0 (delta 0), pack-reused 28737
Receiving objects: 100% (28775/28775), 46.01 MiB | 9.11 MiB/s, done.
Resolving deltas: 100% (19229/19229), done."
```

1. Checkout the 'pic32mzef_wifire' branch.
```
$ cd Espruino
$ git checkout pic32mzef_wifire
```
1. Branch pic32mzef_wifire set up to track remote branch pic32mzef_wifire from origin.
Switched to a new branch 'pic32mzef_wifire'

MPLAB X:
1. In MPLAB X, open the project located in 
./Espruino/targets/pic32mzef/wifire/Harmony-MHC/firmware/Harmony-MHC.X
Ensure that this project is set as your main project.
Launch the MPLAB Harmony Configurator (Tools menu -> Embedded -> MPLAB Harmony Configurator)
Open Configuration: Espruino/targets/pic32mzef/wifire/Harmony-MHC/firmware/src/system_config/wifire/wifire.mhc
On the toolbar on MHC, press the "Generate Code" button and press Generate. MHC should generate new files in the Espruino/targets/pic32mzef/wifire/Harmony-MHC/firmware/src directory.

1. Open 'Makefile' in a text editor and find where the 'MICROCHIP_HARMONY_PATH' variable is set. Here you can set the variable to point to your Harmony installation. Alternatively, you can set up MICROCHIP_HARMONY_PATH as an environment variable.

1. In 'Makefile' search for PIC32MZ_EF_WIFIRE. Here you will find areas where you can add source files, C wrapper source files, and other options that can affect the build process.

TERMINAL:
1. Ensure that the xc32-gcc executable is located in your system PATH. If it is not, add it to your PATH as appropriate for your operating system.
```
$ xc32-gcc --version
xc32-gcc (Microchip Technology) 4.8.3 MPLAB XC32 Compiler v1.42 Build date: Jun  1 2016
```
1. In the Espruino directory containing the Makefile, run the following make command.
```
$ make PIC32MZ_EF_WIFIRE=1 all -j
```
The build may take several minutes. It should end with an elf file being copied to espruino.elf.
Example output: "cp espruino_1v84.1300_pic32mz_ef_wifire.elf espruino.elf"
This ELF file contains the image to be programmed to the target board.

-----------------------------------------------------------
## PROGRAMMING
To program the generated espruino.elf file to the target board:

Connect the debugger to the chipKIT Wi-Fire target board. The RealIce-Debug-WiFire.jpg photo shows the REAL ICE being connected to the WiFire board, but you could also use an MPLAB ICD 3 or a PICkit 3 debugger.

Open the MPLAB X project, Espruino/targets/pic32mzef/Prebuilt-ELF-Loader.X. This project is set to import the espruino.elf file and program it. (Alternatively, you can create a new project and select Microchip Embedded -> Prebuilt (Hex, Loadable Image) Project.)

Make the project your main project and Press the Debug Main Project toolbar button. MPLAB X should import the Espruino.elf file and program it to the target.

Connect WiFire's UART USB port to your PC and open the Espruino Chrome app. Connect to the board in the Espruino app.

You might need to reset the debugger in order to see the output message in the Chrome app.
-----------------------------------------------------------

## Ideas for projects:

- Some useful features such as the Espruino save() function are not implemented for PIC32MZ MCUs. Identify and implement these features.

- Write wrappers for C native functions from MPLAB Harmony. There is a test of the Espruino C wrapper feature located in Espruino/targets/pic32mzef/wifire/libs/test  You might be able to get more advanced features such as Wi Fi working using this method. Note that Harmony source code should not be added to the Espruino project directly. Instead, reference the Harmony files in their install directory.

-----------------------------------------------------------

