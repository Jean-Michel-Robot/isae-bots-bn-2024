#ifndef _STRING_EXT_H
#define _STRING_EXT_H

#ifdef ARDUINO
    #include <Arduino.h>
    typedef String string_t;   

    #define ToString(a) String(a)
#else
    #include <string>
    typedef std::string string_t;

    #define ToString(a) std::to_string(a)
#endif


#endif