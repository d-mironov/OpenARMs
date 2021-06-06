# GPIO API

The GPIO API provides a basic usage for GPIO pins on different GPIO Ports. The features include enabling the GPIO, changing and locking the settings of the GPIO and also writing, toggling and reading 0/1 from the GPIO.

---

### Enabling the GPIO

```c
gpio_err_t GPIO_enable(GPIO_TypeDef *port, const uint8_t pin_num, gpio_mode_t mode);
```

**Arguments:**  

- port -> GPIO port of the pin to enable (e.g GPIOA, GPIOB)
- pin_num -> the pin to enable (pin_num <= 15)
- mode -> The mode of the pin

**Returns:**

- GPIO_PIN_TOO_HIGH - when the pin number is `> 15`
- GPIO_OK - on success

For the port selection refer to the **Definitions and Enums** of the documentation below.

The pin needs to be **bigger than -1** and **smaller than 16**. The reason for that is that the **STM32** boards have 16 pins on each port, starting from 0.

The mode is a `typedef enum` called `gpio_mode_t` The enum is defined as:

```c
typedef enum gpio_mode {
    GPIO_INPUT,
    GPIO_OUTPUT,
    GPIO_ALTERNATE,
    GPIO_ANALOG
} gpio_mode_t;
```

The different modes are self-explanatory. 

Example:

```c
// this will enable PC13 as an output.
GPIO_enable(GPIOC, 13, GPIO_OUTPUT);
```



```c
// this will enable PB8 as an input pin
GPIO_enable(GPIOB, 8, GPIO_INPUT);
```



---

### GPIO settings

```c
gpio_err_t GPIO_settings(GPIO_TypeDef *port, const uint8_t pin_num, const uint8_t speed, const uint8_t pull_up_down);
```

**Arguments:**

- port - Port of GPIO to do settings
- pin_num - Number of pin to do settings
- speed - speed mode of pin
- pull_up_down - pull-up or pull-down selection for GPIO

**Returns:**

- GPIO_PIN_TOO_HIGH - when `pin_num > 15`
- GPIO_OK - on success

For the port selection refer to the **Definitions and Enums** of the documentation below.

Again the `pin_num` must be `< 16`. 

Example:

```c
// this will set the PA10 pin to high speed and will disable the push- and pull-up
GPIO_settings(GPIOA, 10, GPIO_FAST_SPEED, GPIO_NO_PULL_UP_DOWN);
```



---

### GPIO select alternate function

```c
gpio_err_t GPIO_select_alternate(GPIO_TypeDef *port, const uint8_t pin, const uint8_t af);
```

##### **Arguments:**

- port  -  port of GPIO pin
- pin  -  actual GPIO pin
- af  -  Alternate function to select

**Returns:**

- GPIO_PIN_TOO_HIGH  -  when `pin > 15`
- GPIO_ALTERNATE_FUNC_TOO_HIGH  -  when `af > 15`
- GPIO_OK  -  on success

For the `af` argument refer to the **Definitions and Enums** section.

Make sure that your GPIO port and pin is enabled and you selected `GPIO_ALTERNATE` as the mode parameter when you called `GPIO_enable(...)`.

Example:

```c
// this will configure PA2 to use the alternate function 7 (USART)
GPIO_select_alternate(GPIOA, 2, GPIO_AF07);
```



---

### GPIO toggle

```c
gpio_err_t GPIO_toggle(GPIO_TypeDef *port, const uint8_t pin_num);
```

**Arguments:**

- port  -  Port of GPIO pin to toggle
- pin_num  -  pin number of GPIO to toggle

**Returns:**

- GPIO_PIN_TOO_HIGH  -  if `pin_num > 15`
- GPIO_OK  -  on success

This function will toggle the selected pin depending if it is on or off.

Example:

```c
// this will toggle the PC13 pin every 500ms on and off
while (1) {
    GPIO_toggle(GPIOC, 13);
    delayMs(500);
}
```



---

### GPIO write

```c
gpio_err_t GPIO_write(GPIO_TypeDef *port, const uint8_t pin_num, const uint8_t on_off);
```

**Arguments:**

- port  -  Port of GPIO pin to write to

- pin_num  -  pin of GPIO to write to

- on_off  -  turn on or off the GPIO (GPIO_ON, GPIO_OFF)

**Returns:**

- GPIO_PIN_TOO_HIGH  -  if `pin_num > 15`
- GPIO_OK  -  on success

This will write the value `on_off` to the specified pin:

- GPIO_ON  -  will turn on the pin
- GPIO_OFF  -  turns the pin off

Also make sure that your pin is enabled and setup correctly

Example:

```
// this will turn on the PB7 pin
GPIO_write(GPIOB, 7, GPIO_ON);
```

---

### GPIO read digital value

```c
uint8_t GPIO_read_digital(GPIO_TypeDef *port, const uint8_t pin);
```

**Arguments:**

- port  -  Port of GPIO pin to read from
- pin  -  pin number of GPIO to read from

**Returns:**

- `uint8_t`  value read from pin (1 or 0)

