#!/usr/bin/python

## Modify header files to include example files in doxygen comments.
## Outputs a list of example files that do not exist in any headers.
## Will output coordaxes.x3d since it is inlined in other x3d files.

import os, sys
import re
from os.path import join

current_dir = os.getcwd()
examples_dirs = ["../examples/RigidBody", "../examples/softbody"]
count = 0
ignoreList = ["BulletJoints", "BulletCallbacks", "PHYSXCallbacks", "ODECallbacks", "PhysicsEngineParameters", "PhysicsEngineThread", "cooking", "FieldTemplates", "PythonMethods", "SOFACallbacks", "SoftBodyPhysicsEngineThread", "SoftBodyParameters", "Stream", "PhysXCallbacks", "PhysX3Callbacks", "PhysX3Joints"]
examples = []

for examples_dir in examples_dirs:
  if os.path.isdir( examples_dir ):
    temp_examples = []
    print "Handling examples directory " + examples_dir

    for root, dirs, files in os.walk(examples_dir):
      if '.svn' in dirs:
        dirs.remove('.svn')  # don't visit CVS directories
      if 'branches' in dirs:
        dirs.remove('branches')  # don't visit branches directories
      if 'obj' in dirs:
        dirs.remove('obj')  # don't visit obj directories
      if 'vc7' in dirs:
        dirs.remove('vc7')  # don't visit vc7 directories
      if 'vc8' in dirs:
        dirs.remove('vc8')  # don't visit vc8 directories
      if 'vc9' in dirs:
        dirs.remove('vc9')  # don't visit vc9 directories
      for temp_file in files:
        if( temp_file.endswith(".x3d") ):
          temp_examples.append( temp_file.replace( ".x3d", "" ) )
    examples.append( temp_examples )
  else:
    print "The examples/All directory does not exist, this path was used: " + examples_dir

headers_dir = current_dir + "/../include/H3D/H3DPhysics"
if os.path.isdir( headers_dir ):
  print "Handling headers directory " + headers_dir

  for root, dirs, files in os.walk(headers_dir):
    if '.svn' in dirs:
      dirs.remove('.svn')  # don't visit CVS directories
    if 'branches' in dirs:
      dirs.remove('branches')  # don't visit branches directories
    if 'obj' in dirs:
      dirs.remove('obj')  # don't visit obj directories
    if 'vc7' in dirs:
      dirs.remove('vc7')  # don't visit vc7 directories
    if 'vc8' in dirs:
      dirs.remove('vc8')  # don't visit vc8 directories
    if 'vc9' in dirs:
      dirs.remove('v bc9')  # don't visit vc9 directories
    for temp_file in files:
      if( not temp_file.startswith("X3D") and not temp_file.startswith("H3D") and temp_file.endswith(".h") ):
        file_name_no_end = temp_file.replace( ".h", "" )
        if file_name_no_end not in ignoreList :
          name_count = 0
          for temp_examples in examples:
            name_count = name_count + temp_examples.count( file_name_no_end )
          fhandle = open( join( headers_dir, temp_file ), "rb" )
          code = fhandle.read()
          fhandle.close()

          new_string1 = """  ///   - <a href="../../examples/"""

          if code.find( new_string1 ) == -1:
            print "This is a file with no example: " + file_name_no_end
            if file_name_no_end == "CompositeGeometryNode":
              print "CompositeGeometryNode is marked as deprecated and should be removed in the future."
            if name_count > 1:
              print " name_count above 1, there are several examples to choose from for this node."
            count += 1
          
print "This is the total number of files: ", count