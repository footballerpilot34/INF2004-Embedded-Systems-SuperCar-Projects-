/** @file wifi_secrets_example.h
 *
 * @brief Template for the hotspots the car may join (see config.h).
 *
 * @par
 * Copy this file to src/wifi_secrets.h and put in your own hotspots.
 * wifi_secrets.h is in .gitignore, so passwords never reach GitHub.
 * Without it, the car makes its own network (SuperCar-AAI14) instead.
 *
 * @par
 * Add one { "name", "password" } line per hotspot, each ending in a comma
 * and a backslash except the last. The car tries them in this order (or
 * only the one picked on the web page).
 *
 * @par
 * Phone hotspots: use 2.4 GHz (the Pico W has no 5 GHz) and WPA2, or
 * WPA2/WPA3 mixed. On iPhone, turn on "Maximise Compatibility" and keep
 * the Personal Hotspot screen open while the car joins. iPhone names use
 * a curly apostrophe: "Name’s iPhone", not "Name's iPhone".
 *
 * @par
 * After creating or changing wifi_secrets.h, save config.h too (or do a
 * clean build): the build does not notice a newly added file by itself.
 */

#ifndef WIFI_SECRETS_H
#define WIFI_SECRETS_H

#define HOTSPOT_LIST \
    { "My Phone Hotspot",   "hotspot-password"  }, \
    { "Ryan's iPhone",   "ryan2606"   }

#endif /* WIFI_SECRETS_H */

/*** end of file ***/
