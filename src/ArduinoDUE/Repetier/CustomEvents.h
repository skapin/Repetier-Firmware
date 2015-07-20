#ifndef CUSTOM_EVENTS_H_INCLUDED
#define CUSTOM_EVENTS_H_INCLUDED

#define EVENT_WAITING_HEATER(id) {}
#define EVENT_HEATING_FINISHED(id) {}
#define EVENT_TIMER_100MS { check_all_ATU(); chamber.manageTemperatures(); }

#define EVENT_PERIODICAL { eps_manage(); }
#define EVENT_START_UP { init_polybox(); }

#endif //CUSTOM_EVENTS_H_INCLUDED
