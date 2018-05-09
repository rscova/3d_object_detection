3D OBJECT DETECTOR
==============

This code evaluates the differents Keypoints extractors and Descriptors of the PCL library

For use it:
$ cd build
$ cmake ..
$ make
$ evaluator <path of input scene cloud> <path of input object cloud>


If you want to change the descriptor or keypoints you can do it in src/evaluation.cpp

If you want to use FPFH or PFH you have to comment the define STRUCT_DESCRIPTOR in headers/tools.h
