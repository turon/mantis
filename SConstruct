import os
import glob
import string
import sys
from scripts.build_support import *
from scripts.mos_debugging import *

#SConscriptChdir(1)



#local vars

platform   = ARGUMENTS.get('platform', 'mica2')
if platform == 'mica2' or platform == 'micaz' or platform == 'mica2dot' or platform == 'avrdev':
    arch = 'avr'
elif platform == 'telosb':
    arch = 'msp430'
elif platform == 'linux':
    arch = 'linux'
elif platform == 'imote2':
    arch = 'pxa27x'
elif platform == 'microblaze':
    arch = 'microblaze'
else:
    Exit('Error: Undefined platform')

homedir    = os.getcwd ()

localpath  = string.replace(os.environ['PWD'] + '/',
                           homedir + '/', '')
build_path = 'build-' + platform + '/'

debug_arg = ARGUMENTS.get('debug', '0')

use_deputy = ARGUMENTS.get('deputy', '0')

if platform == 'telosb' and debug_arg != '0':
   Exit('Error: NodeMD does not support TELOSB')

# build targets function
def build_src():
    BuildDir (localpath + build_path + 'src', 'src', duplicate = 0)
    SConscript (localpath + build_path + 'src/SConscript', 'env')

    SConscript (localpath + 'SConscript',
                          'env',
                          src_dir = localpath,
                          build_dir = localpath + build_path,
                          duplicate = 0)
    return None


libpaths = '#' + localpath + build_path + 'src/'

# include paths init
cpppath_list = []

# libraries
lib_list = []



# library paths
lib_path_list = []

add_lib_path (lib_path_list, libpaths + 'mos/sys')
add_lib_path (lib_path_list, libpaths + 'mos/dev')
add_lib_path (lib_path_list, libpaths + 'mos/com')
add_lib_path (lib_path_list, libpaths + 'mos/share')
add_lib_path (lib_path_list, libpaths + 'lib')


if arch == 'linux':
    print "Output Target set to Linux (x86)"

    # linux platform includes and libs
    add_include_path (cpppath_list, '#src/mos/kernel/linux/include')
    add_include_path (cpppath_list, '#src/mos/kernel/include')
    add_include_path (cpppath_list, '#src/mos/sys/include')
    add_include_path (cpppath_list, '#src/mos/dev/include')
    add_include_path (cpppath_list, '#src/lib/include')
    add_include_path (cpppath_list, '#src/mos/com/include')
    add_include_path (cpppath_list, '#src/mos/share/include')
    add_include_path (cpppath_list, '/usr/include/netinet')

    add_link_lib (lib_list, 'share')
    add_link_lib (lib_list, 'sys')
    add_link_lib (lib_list, 'com')
    add_link_lib (lib_list, 'moslibs')
    add_link_lib (lib_list, 'kernel')
    add_link_lib (lib_list, 'dev')
    add_link_lib (lib_list, 'pthread')
    add_lib_path (lib_path_list, libpaths + 'mos/kernel/linux')

    # config file for drivers, 
    add_include_path (cpppath_list, '#' + localpath)

    env = Environment(ENV = os.environ,
                      CPPDEFINES = {
                                 'SCONS' : 1,
                                 'PLATFORM_LINUX' : 1,
                                 'ARCH_LINUX' : 1
                                 },
		      CCFLAGS = ['-ggdb', '-Wall'],
                      LINKFLAGS = ['-ggdb', '-Wall'],
		      CPPPATH = cpppath_list,
                      LIBS = lib_list,
                      LIBPATH = lib_path_list,
		      rootdir = homedir + '/',
		      appdir = localpath,
                      builddir = build_path,
                      envparams = '',
		      plat = 'linux',
                      ARCH = 'linux'      
		      )

else:
    # microcontroller includes and libs
    
    add_include_path (cpppath_list, '#src/mos/kernel/include')
    add_include_path (cpppath_list, '#src/mos/sys/include')
    add_include_path (cpppath_list, '#src/mos/com/include')
    add_include_path (cpppath_list, '#src/mos/dev/include')
    add_include_path (cpppath_list, '#src/mos/share/include')
    add_include_path (cpppath_list, '#src/mos/net/include')
    add_include_path (cpppath_list, '#src/lib/include')


    add_link_lib (lib_list, 'gcc')
    add_link_lib (lib_list, 'net')
    add_link_lib (lib_list, 'sys')
    add_link_lib (lib_list, 'share')
    add_link_lib (lib_list, 'com')
    add_link_lib (lib_list, 'moslibs')
    add_link_lib (lib_list, 'dev')
    add_link_lib (lib_list, 'kernel_generic')
    add_link_lib (lib_list, 'kernel')
    add_link_lib (lib_list, 'dev')
    
    add_lib_path (lib_path_list, libpaths + 'mos/net')
    add_lib_path (lib_path_list, libpaths + 'mos/kernel/micro')


    # config file for drivers, 
    add_include_path (cpppath_list, '#'+localpath)

    # options from file
    opts = Options(localpath + 'scons.config')     

    env = Environment(ENV = os.environ,
                      options = opts,
                      CPPDEFINES = {'SCONS' : 1,
                                    'FLOODING_ROUTING' : 1,
                                    'DEBUG_MEMORY' : 1,
                                    },
                      CCFLAGS = ['-Wall',
                                 '-Wno-format',
                                 '-g',
                                 '-Os'
                                 ],
                      LINKFLAGS = ['-Wall',
                                   '-Wno-format',
                                   '-g',
                                   '-Os'
                                   ],
                      PROGSUFFIX = '.elf',
                      
                      CPPPATH = cpppath_list,
                      LIBS = lib_list,
                      LIBPATH = lib_path_list,
                      rootdir = homedir + '/',
                      appdir = localpath,
                      envparams = '',
                      builddir = build_path,
                      # set debug here temporarily,TODO: set it at compile time
                      debug = debug_arg,
                      dbcode_procedure = 1
                      )

    env.Append(CPPDEFINES = {'ARCH_MICRO' : 1})

    if arch == 'msp430':
        print "Output Target set to TELOSB"

        env.Append(CPPPATH = '#src/mos/kernel/micro/include')
        env.Append(CPPPATH = '#src/mos/kernel/msp430/include')
        #env.Append(CPPPATH = '#src/mos/kernel/msp430/boot')
        env.Append(LIBPATH = libpaths + 'mos/kernel/msp430')

        env.Append(CPPDEFINES = {'PLATFORM_TELOSB' : 1})
        env.Append(CPPDEFINES = {'CLOCK_SPEED_3_68' : 1})
        env.Append(CPPDEFINES = {'ARCH_MSP430' : 1})
	env.Append(CPPDEFINES = {'PLATFORM_SUPPORTS_CC2420' : 1})

        env.Append(CCFLAGS = '-mmcu=msp430x1611')
        env.Append(LINKFLAGS = '-mmcu=msp430x1611')  
	
	if use_deputy == '0': 
            env['CC'] = 'msp430-gcc'
	else:
            env['CC'] = 'deputy-msp'
            env.Append(CPPDEFINES = {'__MSP430_1611__' : 1})
            env.Append(LINKFLAGS = '--nolib')
        
        
        env['plat'] = 'telosb'
        env['RANLIB'] = 'msp430-ranlib'
        env['OBJCOPY'] = 'msp430-objcopy'
        env['ARCH'] = 'msp430'

    if arch == 'microblaze':
       print "Output Target set to Microblaze"

       env.Append(CPPPATH = '#src/mos/kernel/micro/include')
       env.Append(CPPPATH = '#src/mos/kernel/microblaze/include')
       env.Append(CPPPATH = '/usr/include')
       env.Append(CPPPATH = '#../nodeblaze/microblaze_0/include')
       env.Append(LIBPATH = libpaths + 'mos/kernel/microblaze')
       env.Append(LIBPATH = libpaths + '#../nodeblaze/microblaze_0/lib')

       env.Append(CPPDEFINES = {'PLATFORM_MICROBLAZE' : 1})
       env.Append(CPPDEFINES = {'CLOCK_SPEED_4_0' : 1})
       env.Append(CPPDEFINES = {'ARCH_MICROBLAZE' : 1})
       env.Append(CPPDEFINES = {'PLATFORM_SUPPORTS_CC2420' :1})       

       env.Append(CCFLAGS = '-mcpu=v5.00.c')
       env.Append(CCFLAGS = '-mno-xl-soft-mul')
       env.Append(CCFLAGS = '-mxl-pattern-compare')
       env.Append(LINKFLAGS = '-mmcu=v5.00c -Wl,-T -Wl,#../nodeblaze/NodeBlaze_linker_script.ld ')  

       env['CC'] = 'mb-gcc'
       env['plat'] = 'microblaze'
       env['RANLIB'] = 'mb-ranlib'
       env['OBJCOPY'] = 'mb-objcopy'
       env['ARCH'] = 'microblaze'


    if arch == 'avr':
        env.Append(CPPPATH = '#src/mos/kernel/micro/include')
        env.Append(CPPPATH = '#src/mos/kernel/avr/include')
        env.Append(CPPPATH = '#src/mos/kernel/avr/boot/include')
        env.Append(LIBPATH = libpaths + 'mos/kernel/avr')

        env.Append(CPPDEFINES = {'ARCH_AVR' : 1})


        env.Append(CCFLAGS = '-mmcu=atmega128')
        env.Append(LINKFLAGS = '-mmcu=atmega128')
        
        if use_deputy == '0':
            env['CC'] = 'avr-gcc'
        else:
            env['CC'] = 'deputy-avr'
            env.Append(LINKFLAGS = '--nolib')

        env['RANLIB'] = 'avr-ranlib'
        env['OBJCOPY'] = 'avr-objcopy'
        env['ARCH'] = 'avr'
                
        if platform == 'mica2':
            print "Output Target set to MICA2"
            env.Append(CPPDEFINES = {'PLATFORM_MICA2' : 1})
            env.Append(CPPDEFINES = {'CLOCK_SPEED_7_37' : 1})
            env.Append(CPPDEFINES = {'PLATFORM_MICA_ANY' : 1})
            env['plat'] = 'mica2'

        if platform == 'micaz':
            print "Output Target set to MICAZ"
            env.Append(CPPDEFINES = {'PLATFORM_MICAZ' : 1})
            env.Append(CPPDEFINES = {'CLOCK_SPEED_7_37' : 1})
            env.Append(CPPDEFINES = {'PLATFORM_MICA_ANY' : 1})
	    env.Append(CPPDEFINES = {'PLATFORM_SUPPORTS_CC2420' : 1})

            env['plat'] = 'micaz' 
    
        if platform == 'mica2dot':
            print "Output Target set to MICA2DOT"
            env.Append(CPPDEFINES = {'PLATFORM_MICA2DOT' : 1})
            env.Append(CPPDEFINES = {'CLOCK_SPEED_4_0' : 1})
            env.Append(CPPDEFINES = {'PLATFORM_MICA_ANY' : 1})
            env['plat'] = 'mica2dot'

        if platform == 'avrdev':
            print "Output Target set to AVRDEV"
            env.Append(CPPDEFINES = {'PLATFORM_AVRDEV' : 1})
            env.Append(CPPDEFINES = {'CLOCK_SPEED_3_68' : 1})
            env['plat'] = 'avrdev'
            env['envparams'] = '--sbaud B38400 '
    if arch == 'pxa27x':
        print "Output Target set to IMOTE2"
        
        env.Append(CPPPATH = '#src/mos/kernel/micro/include')
        env.Append(CPPPATH = '#src/mos/kernel/pxa27x/include')
	
        env.Append(LIBPATH = libpaths + 'mos/kernel/pxa27x')

        env.Append(CPPDEFINES = {'PLATFORM_IMOTE2' : 1})
        env.Append(CPPDEFINES = {'CLOCK_SPEED_3_68' : 1})
        env.Append(CPPDEFINES = {'ARCH_PXA27X' : 1})

        #env.Append(CCFLAGS = '')
	env.Append(ASFLAGS = '-mcpu=iwmmxt -mfpu=softfpa')

        env['LINKFLAGS'] = 'src/mos/kernel/pxa27x/support.o --cref -T imote2.ld -nostartfiles'

	env['AS'] = 'xscale-elf-gcc -c'
        env['CC'] = 'xscale-elf-gcc'
        env['plat'] = 'imote2'
        env['RANLIB'] = 'xscale-elf-ranlib'
        env['OBJCOPY'] = 'xscale-elf-objcopy'
        env['ARCH'] = 'pxa27x'
        
if os.getenv('CC') != None:
   env['CC'] = os.getenv('CC')
if os.getenv('CFLAGS') != None:
   env['CCFLAGS'] = os.getenv('CFLAGS')
if os.getenv('ASFLAGS') != None:
   env['LINKFLAGS'] = os.getenv('ASFLAGS')

if debug_arg != '0':
    print "Build set to DEBUG MODE"
    env.Append(CPPDEFINES = {'MOS_DEBUG' : 1})


env['loadparams'] = ARGUMENTS.get('loadparams', '')
env['telosport'] = ARGUMENTS.get('port', '')

# define builder for avr-objcopy
objcopy = Builder(action = obj_copy_function)
env.Append(BUILDERS = {'ObjCopy' : objcopy})

debugproc = Builder(action = mos_debug_preprocessor)
env.Append(BUILDERS = {'Debug' : debugproc})

# set build targets
if os.access(localpath + 'optsconfig.h', os.F_OK):
    targets = build_src()
else:
    Exit('Error: Current directory does not contain an optsconfig.h file')

#SourceSignatures('timestamp')

Export ('env')


