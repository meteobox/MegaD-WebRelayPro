# MegaD-WebRelayPro
<pre>
------ Build started: Project: MegaDubnino, Configuration: Debug AVR ------
Build started.
Project "MegaDubnino.cproj" (default targets):
Target "PreBuildEvent" skipped, due to false condition; ('$(PreBuildEvent)'!='') was evaluated as (''!='').
Target "CoreBuild" in file "C:\Program Files (x86)\Atmel\Studio\7.0\Vs\Compiler.targets" from project "C:\projects\MegaD-WebRelayPro\GITHUB-MegaD-WebRelayPro\GccApplication1\MegaDubnino.cproj" (target "Build" depends on it):
	Task "RunCompilerTask"
		Shell Utils Path C:\Program Files (x86)\Atmel\Studio\7.0\shellUtils
		C:\Program Files (x86)\Atmel\Studio\7.0\shellUtils\make.exe all --jobs 8 --output-sync 
		Building file: .././hw_dht.c
		Invoking: AVR/GNU C Compiler : 5.4.0
		"C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-gcc.exe"  -x c -funsigned-char -funsigned-bitfields -DDEBUG  -I"C:\Program Files (x86)\Atmel\Studio\7.0\Packs\atmel\ATmega_DFP\1.1.130\include"  -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -g2 -Wall -mmcu=atmega328p -B "C:\Program Files (x86)\Atmel\Studio\7.0\Packs\atmel\ATmega_DFP\1.1.130\gcc\dev\atmega328p" -c -std=gnu99 -MD -MP -MF "hw_dht.d" -MT"hw_dht.d" -MT"hw_dht.o"   -o "hw_dht.o" ".././hw_dht.c" 
		Finished building: .././hw_dht.c
		Building file: .././enc28j60.c
		Invoking: AVR/GNU C Compiler : 5.4.0
		"C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-gcc.exe"  -x c -funsigned-char -funsigned-bitfields -DDEBUG  -I"C:\Program Files (x86)\Atmel\Studio\7.0\Packs\atmel\ATmega_DFP\1.1.130\include"  -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -g2 -Wall -mmcu=atmega328p -B "C:\Program Files (x86)\Atmel\Studio\7.0\Packs\atmel\ATmega_DFP\1.1.130\gcc\dev\atmega328p" -c -std=gnu99 -MD -MP -MF "enc28j60.d" -MT"enc28j60.d" -MT"enc28j60.o"   -o "enc28j60.o" ".././enc28j60.c" 
		Finished building: .././enc28j60.c
		Building file: .././websrv_help_functions.c
		Invoking: AVR/GNU C Compiler : 5.4.0
		"C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-gcc.exe"  -x c -funsigned-char -funsigned-bitfields -DDEBUG  -I"C:\Program Files (x86)\Atmel\Studio\7.0\Packs\atmel\ATmega_DFP\1.1.130\include"  -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -g2 -Wall -mmcu=atmega328p -B "C:\Program Files (x86)\Atmel\Studio\7.0\Packs\atmel\ATmega_DFP\1.1.130\gcc\dev\atmega328p" -c -std=gnu99 -MD -MP -MF "websrv_help_functions.d" -MT"websrv_help_functions.d" -MT"websrv_help_functions.o"   -o "websrv_help_functions.o" ".././websrv_help_functions.c" 
		Finished building: .././websrv_help_functions.c
		Building file: .././ip_arp_udp_tcp.c
		Invoking: AVR/GNU C Compiler : 5.4.0
		"C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-gcc.exe"  -x c -funsigned-char -funsigned-bitfields -DDEBUG  -I"C:\Program Files (x86)\Atmel\Studio\7.0\Packs\atmel\ATmega_DFP\1.1.130\include"  -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -g2 -Wall -mmcu=atmega328p -B "C:\Program Files (x86)\Atmel\Studio\7.0\Packs\atmel\ATmega_DFP\1.1.130\gcc\dev\atmega328p" -c -std=gnu99 -MD -MP -MF "ip_arp_udp_tcp.d" -MT"ip_arp_udp_tcp.d" -MT"ip_arp_udp_tcp.o"   -o "ip_arp_udp_tcp.o" ".././ip_arp_udp_tcp.c" 
		Finished building: .././ip_arp_udp_tcp.c
		Building file: .././main.c
		Invoking: AVR/GNU C Compiler : 5.4.0
		"C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-gcc.exe"  -x c -funsigned-char -funsigned-bitfields -DDEBUG  -I"C:\Program Files (x86)\Atmel\Studio\7.0\Packs\atmel\ATmega_DFP\1.1.130\include"  -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -g2 -Wall -mmcu=atmega328p -B "C:\Program Files (x86)\Atmel\Studio\7.0\Packs\atmel\ATmega_DFP\1.1.130\gcc\dev\atmega328p" -c -std=gnu99 -MD -MP -MF "main.d" -MT"main.d" -MT"main.o"   -o "main.o" ".././main.c" 
		Finished building: .././main.c
		Building target: MegaDubnino.elf
		Invoking: AVR/GNU Linker : 5.4.0
		"C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-gcc.exe" -o MegaDubnino.elf  enc28j60.o hw_dht.o ip_arp_udp_tcp.o main.o websrv_help_functions.o   -Wl,-Map="MegaDubnino.map" -Wl,--start-group -Wl,-lm  -Wl,--end-group -Wl,--gc-sections -mmcu=atmega328p -B "C:\Program Files (x86)\Atmel\Studio\7.0\Packs\atmel\ATmega_DFP\1.1.130\gcc\dev\atmega328p"  
		Finished building target: MegaDubnino.elf
		"C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-objcopy.exe" -O ihex -R .eeprom -R .fuse -R .lock -R .signature -R .user_signatures  "MegaDubnino.elf" "MegaDubnino.hex"
		"C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-objcopy.exe" -j .eeprom  --set-section-flags=.eeprom=alloc,load --change-section-lma .eeprom=0  --no-change-warnings -O ihex "MegaDubnino.elf" "MegaDubnino.eep" || exit 0
		"C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-objdump.exe" -h -S "MegaDubnino.elf" > "MegaDubnino.lss"
		"C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-objcopy.exe" -O srec -R .eeprom -R .fuse -R .lock -R .signature -R .user_signatures "MegaDubnino.elf" "MegaDubnino.srec"
		"C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-size.exe" "MegaDubnino.elf"
		   text	   data	    bss	    dec	    hex	filename
		  28848	   1159	   1269	  31276	   7a2c	MegaDubnino.elf
	Done executing task "RunCompilerTask".
	Task "RunOutputFileVerifyTask"
				Program Memory Usage 	:	29200 bytes   89,1 % Full
				Data Memory Usage 		:	1621 bytes   79,2 % Full
				EEPROM Memory Usage 	:	807 bytes   78,8 % Full
	Done executing task "RunOutputFileVerifyTask".
Done building target "CoreBuild" in project "MegaDubnino.cproj".
Target "PostBuildEvent" skipped, due to false condition; ('$(PostBuildEvent)' != '') was evaluated as ('' != '').
Target "Build" in file "C:\Program Files (x86)\Atmel\Studio\7.0\Vs\Avr.common.targets" from project "C:\projects\MegaD-WebRelayPro\GITHUB-MegaD-WebRelayPro\GccApplication1\MegaDubnino.cproj" (entry point):
Done building target "Build" in project "MegaDubnino.cproj".
Done building project "MegaDubnino.cproj".

Build succeeded.
========== Build: 1 succeeded or up-to-date, 0 failed, 0 skipped ==========

