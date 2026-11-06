/** @file wifi_secrets.h
 *
 * @brief The hotspots the car may join (see config.h).
 *
 * @par
 * Add one { "name", "password" } line per hotspot, each ending in a comma
 * and a backslash except the last. The car tries them in this order (or
 * only the one picked on the web page). Without this file, or with
 * HOTSPOT_LIST deleted, the car makes its own network instead.
 *
 * @par
 * Phone hotspots: use 2.4 GHz (the Pico W has no 5 GHz) and WPA2, or
 * WPA2/WPA3 mixed. On iPhone, turn on "Maximise Compatibility" and keep
 * the Personal Hotspot screen open while the car joins. iPhone names use
 * a curly apostrophe: "Ryan’s iPhone", not "Ryan's iPhone".
 */

#ifndef WIFI_SECRETS_H
#define WIFI_SECRETS_H

#define HOTSPOT_LIST \
    { "Ryan’s iPhone",      "ryan2606"          }, \
    { "Friend's Hotspot",   "friend-password"   }

#endif /* WIFI_SECRETS_H */

/*** end of file ***/
