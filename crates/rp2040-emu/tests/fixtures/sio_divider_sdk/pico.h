// Assembly-only SDK configuration. Arithmetic and register definitions come
// from the pinned Pico SDK, without changes to its divider code or helpers.
#define WRAPPER_FUNC_NAME(x) x
#define SECTION_NAME(x) .text
#define RAM_SECTION_NAME(x) .text
#define PICO_DIVIDER_DISABLE_INTERRUPTS 0
#define PICO_DIVIDER_CALL_IDIV0 0
#define PICO_DIVIDER_CALL_LDIV0 0
