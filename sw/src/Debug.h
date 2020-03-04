
#ifndef DEBUG_H
#define DEBUG_H

#ifdef __cplusplus
	extern "C" {
#endif

#DEFINE DEBUG_MSG(msg) Serial.Print(msg);Serial.Print(" on ");Serial.Print(__func__);Serial.Print(":");Serial.Print(__FILE__);Serial.Print(":");Serial.Print(__LINE__);

#ifdef __cplusplus
	}
#endif

#endif