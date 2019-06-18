# Changed the name of an TrueStudio project
import os
import sys

def inplace_change(filename, old_string, new_string):
    # Safely read the input filename using 'with'
    with open(filename) as f:
        s = f.read()
        if old_string not in s:
            print '"{old_string}" not found in {filename}.'.format(**locals())
            return

    # Safely write the changed content, if found in the file
    with open(filename, 'w') as f:
        print 'Changing "{old_string}" to "{new_string}" in {filename}'.format(**locals())
        s = s.replace(old_string, new_string)
        f.write(s)

# Getting the current work directory (cwd)
dir_abs = os.getcwd()
dir_abs_split=dir_abs.split('\\')
n = len(dir_abs_split)
dst_dir = dir_abs_split[n-1]
new_name = dst_dir
print dir_abs

truestudio_version = "TrueSTUDIO for STM32 9.3.0"

# Get original name
#src_dir = os.listdir(ts_dir)[0]
#old_name = src_dir
#print "old_name: " + old_name

mxproject_filename = dir_abs + "\.mxproject"
with open(mxproject_filename) as f:
    content = f.readlines()
second_line = content[1]
#print second_line
second_line_split=second_line.split('/')
n=len(second_line_split)
old_name = second_line_split[n-2]
print "old_name: " + old_name
print "new_name: " + new_name

ioc_filename_old = dir_abs + "\\" + old_name + ".ioc"
ioc_filename_new = dir_abs + "\\" + new_name + ".ioc"

ts_dir = dir_abs + "\TrueSTUDIO"

ts_name_old = ts_dir + "\\" + old_name
ts_name_new = ts_dir + "\\" + new_name

elf_launch_old = ts_dir + "\\" + new_name + "\\" + old_name + ".elf.launch"
elf_launch_new = ts_dir + "\\" + new_name + "\\" + new_name + ".elf.launch"

cproject = ts_dir + "\\" +  new_name + "\.cproject"
project = ts_dir + "\\" +  new_name + "\.project"

print "Change path in " + project

new_path = "PARENT-2-PROJECT_LOC"
old_path = dir_abs.replace("\\", "/")
old_path = old_path.replace("c:", "C:")
print old_path
print new_path

if os.path.isfile(project):
    # file exists
    print "Modify file " + project
    inplace_change(project, old_path, new_path)

# .cproject
if os.path.isfile(cproject):
    # file exists
    print "Modify file " + cproject
    inplace_change(cproject, "TrueSTUDIO for STM32 9.2.0", truestudio_version)
    inplace_change(cproject, "TrueSTUDIO for STM32 9.1.0", truestudio_version)
    inplace_change(cproject, "TrueSTUDIO for STM32 9.0.1", truestudio_version)
    inplace_change(cproject, "TrueSTUDIO for STM32 9.0.0", truestudio_version)

# .elf.launch
if os.path.isfile(elf_launch_new):
    # file exists
    print "Modify file " + elf_launch_new
    inplace_change(elf_launch_new, "TrueSTUDIO for STM32 9.2.0", truestudio_version)
    inplace_change(elf_launch_new, "TrueSTUDIO for STM32 9.1.0", truestudio_version)
    inplace_change(elf_launch_new, "TrueSTUDIO for STM32 9.0.1", truestudio_version)
    inplace_change(elf_launch_new, "TrueSTUDIO for STM32 9.0.0", truestudio_version)

if (new_name == old_name):
    print "Nothing else to change"
    sys.exit(0)
    

print "Rename directories and files:"

#os.rename(src, dst)
if os.path.isdir(ts_name_old):
    # dir exists
    print "Rename directory " + ts_name_old + " to " + ts_name_new
    os.rename(ts_name_old, ts_name_new)

#os.rename(src, dst)
if os.path.isfile(ioc_filename_old):
    # file exists
    print "Rename file " + ioc_filename_old + " to " + ioc_filename_new
    os.rename(ioc_filename_old, ioc_filename_new)

if os.path.isfile(elf_launch_old):
    # file exists
    print "Rename file " + elf_launch_old + " to " + elf_launch_new
    os.rename(elf_launch_old, elf_launch_new)

print "Replace strings in files:"

# .cproject
if os.path.isfile(cproject):
    # file exists
    print "Modify file " + cproject
    inplace_change(cproject, old_name, new_name)
#    inplace_change(cproject, "TrueSTUDIO for STM32 9.2.0", truestudio_version)
#    inplace_change(cproject, "TrueSTUDIO for STM32 9.1.0", truestudio_version)
#    inplace_change(cproject, "TrueSTUDIO for STM32 9.0.1", truestudio_version)
#    inplace_change(cproject, "TrueSTUDIO for STM32 9.0.0", truestudio_version)

# .project
if os.path.isfile(project):
    # file exists
    print "Modify file " + project
    inplace_change(project, old_name, new_name)
    inplace_change(project, old_path, new_path)

# .ioc
if os.path.isfile(ioc_filename_new):
    # file exists
    print "Modify file " + ioc_filename_new
    inplace_change(ioc_filename_new, old_name, new_name)

# .elf.launch
if os.path.isfile(elf_launch_new):
    # file exists
    print "Modify file " + elf_launch_new
    inplace_change(elf_launch_new, old_name, new_name)
#    inplace_change(elf_launch_new, "TrueSTUDIO for STM32 9.2.0", truestudio_version)
#    inplace_change(elf_launch_new, "TrueSTUDIO for STM32 9.1.0", truestudio_version)
#    inplace_change(elf_launch_new, "TrueSTUDIO for STM32 9.0.1", truestudio_version)
#    inplace_change(elf_launch_new, "TrueSTUDIO for STM32 9.0.0", truestudio_version)

# .mxproject
if os.path.isfile(mxproject_filename):
    # file exists
    print "Modify file " + mxproject_filename
    inplace_change(mxproject_filename, old_name, new_name)
