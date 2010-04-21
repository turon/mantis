
import os
import string


def add_include_path (list, toadd):
    i = string.find (toadd, '-I')
    if i != -1:
        toadd = toadd[i + 2:]
    list.append (toadd)
    return

def add_link_lib (list, toadd):
    i = string.find (toadd, '-l')
    if i != -1:
        toadd = toadd[i + 2:]
    list.append (toadd)
    return

def add_lib_path (list, toadd):
    i = string.find (toadd, '-L')
    if i != -1:
        toadd = toadd[i + 2:]
    list.append (toadd)
    return

#def run_pkg_config (list, command):
#    str_list = os.popen (command).read ().split ()
#    if len (str_list) == 0:
#        print "Command: '" + command + "' failed, exiting"
#        sys.exit (-1)
#    for substr in str_list:
#        list.append (substr.strip ('-I'))
#    return


def listfind(f, seq):
  for item in seq:
    if f == item: 
      return True
  return False

def strip_path_from_files (file_list):
    for i, file in enumerate (file_list):
        j = string.rfind (file, '/')
        file_list[i] = file[j + 1:]
    return file_list

#def strip_lib_leading_l (lib_list):
#    for i, lib in enumerate (lib_list):
#        j = string.find (lib, 'l')
#        lib_list[i] = lib[j + 1:]
#    return lib_list


def obj_copy_function(target, source, env):
    if env['ARCH'] == 'avr':
       outtarget = 'srec'
    elif env['ARCH'] == 'pxa27x':
       outtarget = 'binary'
    else:
       outtarget = 'srec'

    target_file_path = str(target[0])
    j = string.rfind (target_file_path, '/')
    target_file = target_file_path[j + 1:]

    source_file_path = str(source[0])
    j = string.rfind (target_file_path, '/')
    target_file = target_file_path[j + 1:]
    
    temp = env.Execute(env.Action(env['OBJCOPY'] + ' --output-target=' + outtarget + ' '
                      + source_file_path + ' '
                      + target_file))
    temp2 = env.Execute(env.Action('mv ' + target_file + ' ' + target_file_path))
    env.Default(temp)
    env.Default(temp2)




def build_app_function(env, app_names_list, app_sources_list, loadstat):
    # SCons compilation

    appfiles = []
    index = 0;
    file_to_load = None
    for i in app_sources_list:
        appname = string.replace(app_names_list[index], '.elf', '')

        if env['debug'] == '1':
            env.Default(env.Debug(appname + '.debug.c', app_sources_list[index]))
            #app_names_list[index] = appname + '.debug.c'
            appfile = env.Program (app_names_list[index], appname + '.debug.c')
        else:
            appfile = env.Program (app_names_list[index], i)
            
        if env['ARCH'] == 'avr':
            #add avr-object copy function to compile srec
            env.Default(env.ObjCopy( appname + '.srec', app_names_list[index]))
	elif env['ARCH'] == 'pxa27x':
	    env.Default(env.ObjCopy( appname + '.bin', app_names_list[index]))     
        else:
            env.Default(appfile)  
        index = index + 1;
        
        if loadstat != '0':
            if appname == loadstat:
                file_to_load = appname
                
    if env['ARCH'] == 'avr':
        if file_to_load != None:
            temp = env.Command('load the image',
                               [],
                               'mos_shell ' + env['envparams'] + env['loadparams'] +
                               ' -p ' + env['appdir'] + env['builddir'] +
                               file_to_load + '.srec')
            env.Depends(temp, file_to_load + '.srec')
            env.Default(temp)
        else:
            if loadstat != '0':
                env.Exit('Error: Invalid application to load')
            else:
                temp_app_name = string.replace(app_names_list[0], '.elf', '')
                load = env.Command('load the image',
                                   [],
                                   'mos_shell ' + env['envparams'] +
                                   env['loadparams'] + ' -p ' + env['appdir'] +
                                   env['builddir'] +  temp_app_name + '.srec')
                env.Depends(load, temp_app_name + '.srec')
                env.Alias('load', load)
            
    if env['ARCH'] == 'msp430':
        if env['telosport'] == '':
            env['telosport'] = '/dev/ttyUSB0'
            
        tinstall = 'bsl.py --telosb --masserase -c ' + env['telosport'] + ' && bsl.py --telosb -c ' + env['telosport'] + ' --program '
        tmos_shell = 'mos_shell --sdev ' + env['telosport'] + ' -n'
        
        if file_to_load != None:
            temp = env.Command('load the image',
                               [],
                               tinstall + env['appdir'] + env['builddir'] + file_to_load + '.elf' + ' && ' + tmos_shell)

            env.Depends(temp, file_to_load + '.elf')
            env.Default(temp)
        else:
            if loadstat != '0':
                env.Exit('Error: Invalid application to load')
            else:
                temp_app_name = app_names_list[0]
                load = env.Command('load the image',
                                   [],
                                   tinstall + env['appdir'] + env['builddir'] + temp_app_name + ' && ' + tmos_shell)
            
                env.Depends(load, temp_app_name)
                env.Alias('load', load)
