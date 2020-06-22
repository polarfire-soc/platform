# PolarFire SoC Configuration Generator
This is a utility to generate embedded software header files from information 
supplied by Libero from the Libero design. Libero supplies the information in 
the form of an xml file. This can be found in the Libero component subdirectory
e.g: /component/work/PFSOC_MSS_C0/PFSOC_MSS_C0_0

# Table of contents
1. [ Content ](#content)
2. [ Preparation ](#prep)
3. [ Steps to generate embedded software files ](#initial)
4. [ Integrate <hardware> into Embedded Software Project ](#Int)


## 1. Content <a name="content"></a>
The table below describes the content of the PolarFire SoC Configuration Generator 
directory

| File                                          | Description       |       
| :-------------------------------------------- |:------------------| 
| readme.md                                     | This file.        |
| mpfs_configuration_generator.py               | Python script. Takes .xml as argument, produces output for embedded software.|   
| gen_hw_headers.bat                            | Batch script for use on windows command line. Edit with the xml file you wish to use.|    
| gen_hw_headers_lin.sh                         | Bash script for use in Linux terminal. Edit with the xml file you wish to use.|    
| ref_xml/mpfs_hw_description_reference.xml     | Example Libero .xml file. |    

##
~~~
   +---------+
   | root    +-+-->readme.md
   +---------+ +-->mpfs_configuration_generator.py
               +-->gen_hw_headers_lin.sh
               +-->gen_hw_headers.bat
               |
               |       +-----------+
               +------>| ref_xml   +-->mpfs_hw_ref_ddr3_100Mhz_ext_clk.xml
               |       +-----------+
               |     +-----------------------------------------------+
               |     | +-----------+         Output:                 |
               +------>| hardware  +-->Created header files          |
                     | +-----------+                                 |
                     +-----------------------------------------------+
~~~

## 2. Preparation <a name="prep"></a>
Python must be present on the computer to run the PolarFire SoC Configuration Generator.
The Python script will run on Python version 2 or 3.
There is an example script for Linux called <gen_hw_headers_lin.sh>. If using or creating your own 
please make sure it is given permission to execute by running the following command 'chmod +x <scriptName.sh>'

## 3. Steps to generate embedded software files <a name="initial"></a>
Please follow the recommended steps
1. Copy the Libero generated or hand crafted xml file to the ref_xml directory
2. Delete or rename the subdirectory #hardware# as it will be overwritten
3. Edit the gen_hw_headers.bat or gen_hw_headers_lin.sh if using linux with source xml name
4. The subdirectory #hardware will be created containing content for embedded software. 

#### Example generating from a command line in windows
~~~~
C:\mpfs-bare-metal-sw-config-generator\lib>py -3 mpfs_configuration_generator.py ref_xml/pf_soc_hw_description_reference.xml
generate header files for Embedded software project
pfsoc-baremetal-software-cfg-gen.py
python interpreter details: sys.version_info(major=3, minor=7, micro=4, releaselevel='final', serial=0)
python interpreter running is version 3
output header files created in hardware/ directory
C:\mpfs-bare-metal-sw-config-generator\lib>
~~~~

#### Example generating from a command line in Linux
~~~~
vagrant@ubuntu-xenial:/home/mpfs-bare-metal-sw-config-generator/lib$ python3 mpfs_configuration_generator.py ref_xml/pf_soc_hw_description_reference.xml
generate header files for Embedded software project
mpfs_configuration_generator.py
python interpreter details: sys.version_info(major=3, minor=5, micro=2, releaselevel='final', serial=0)
python interpreter running is version 3
output header files created in hardware/ directory
vagrant@ubuntu-xenial:/home/mpfs-bare-metal-sw-config-generator/lib$ 
~~~~

## 4. Integrate <hardware> folder into Embedded Software Project <a name="Int"></a>

This section describes how to integrate the #hardware folder into an embedded software project.

##### Project directory structure, showing where hardware folder sits.
~~~
   +---------+      +-----------+                      +---------+
   | src     +----->|application|                  +-->|hardware |
   +---------+  |   +-----------+                  |   +---------+
                |                                  |
                |   +-----------+                  |   +---------+
                +-->|modules    |                  +-->|linker   |
                |   +-----------+                  |   +---------+
                |                                  |
                |   +-----------+     +---------+  |   +---------+
                +-->|platform   +---->|config   +--+-->|software |
                    +-----------+  |  +---------+      +---------+
                                   |
                                   |  +---------+
                                   +->|drivers  |
                                   |  +---------+
                                   |
                                   |  +---------+      +----------+
                                   +->|hal      +----->|mss_uart  |
                                   |  +---------+      +----------+              
                                   |
                                   |  +---------+      +----------+
                                   +->|mpfs_hal +----->|nwc       |
                                      +---------+      +----------+
~~~


Please follow the recommended steps
1. Delete the #platform/config/hardware folder in the Embedded Software project.
2. Copy the generated subdirectory <hardware> into the project #platform/config/ folder


