// Arduino requires a .ino matching the folder name; the sketch itself lives in
// tab5_map.cpp so that the same file compiles under ESP-IDF, where .ino is not
// a recognised source type.
//
// Nothing needs to be here. Arduino compiles every .cpp in the sketch folder,
// and setup()/loop() are found by the linker wherever they are defined.
