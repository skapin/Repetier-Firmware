#ifndef CUSTOM_EVENTS_H_INCLUDED
#define CUSTOM_EVENTS_H_INCLUDED

#define EVENT_WAITING_HEATER(id) {}
#define EVENT_HEATING_FINISHED(id) {}
#define EVENT_TIMER_100MS {}

/*#define EVENT_MCODE_PARSING(gcode_struct, gcode_number) { PARSE_POLYBOX_MCODE(gcode_struct, gcode_number);}

void PARSE_POLYBOX_MCODE(GCode *com, uint16_t gcode_number);
*/
#endif //CUSTOM_EVENTS_H_INCLUDED
