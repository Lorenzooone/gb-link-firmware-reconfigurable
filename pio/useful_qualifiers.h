#ifndef USEFUL_QUALIFIERS_H_
#define USEFUL_QUALIFIERS_H_

// Implementation specific attribute definition to make sure
// a function executes on time. If that's not needed, comment this line away.
// On Pico, this makes it so the function resides in RAM instead of Flash.
// IMPLEMENTATION-SPECIFIC
#define TIME_SENSITIVE(x) __attribute__((section(".time_critical." #x))) x

#ifndef TIME_SENSITIVE
#define TIME_SENSITIVE(x) x
#endif

#endif /* USEFUL_QUALIFIERS_H_ */
