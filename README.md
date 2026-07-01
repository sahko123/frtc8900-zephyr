# frtc8900-zephyr

Out-of-tree Zephyr RTOS driver for the **Nyfea FRTC8900** I2C real-time clock
module. Structured as a West module so it can be added to any Zephyr project as
a dependency, and laid out to match Zephyr upstream conventions for a future
`drivers/rtc/` contribution.

## Device overview

The FRTC8900 is a compact RTC with a built-in temperature-compensated 32.768 kHz
oscillator (DTCXO). It communicates over I2C at up to 400 kHz and provides:

- Full calendar (second, minute, hour, day-of-week, date, month, year 2000–2099)
- Alarm interrupt (minute, hour, and day-of-month or day-of-week)
- Fixed-cycle wakeup timer
- Time-update interrupt (every second or every minute)
- Battery backup switchover with voltage-loss detection (VLF flag)
- On-chip temperature sensor

I2C slave address: **0x32** (fixed, 7-bit).

## Supported Zephyr RTC API features

| Feature | Supported |
|---|---|
| `rtc_set_time` / `rtc_get_time` | Yes |
| `rtc_alarm_set_time` / `rtc_alarm_get_time` | Yes (`CONFIG_RTC_ALARM`) |
| `rtc_alarm_is_pending` | Yes |
| `rtc_alarm_set_callback` (interrupt-driven) | Yes (requires `int-gpios`) |
| `rtc_update_set_callback` | Not yet |
| Calibration | Not applicable (DTCXO handles compensation internally) |

## Devicetree

### Binding

```
compatible: "nyfea,frtc8900"
```

### Example node

```dts
&i2c0 {
    frtc8900: frtc8900@32 {
        compatible = "nyfea,frtc8900";
        reg = <0x32>;
        /* /INT pin — active low, open drain. Add pull-up on PCB or enable GPIO_PULL_UP. */
        int-gpios = <&gpio0 0 (GPIO_ACTIVE_LOW | GPIO_PULL_UP)>;
    };
};
```

`int-gpios` is optional. Without it, alarm callbacks are not available but
`rtc_alarm_is_pending()` (polling) still works.

## West module setup

Add to your project's `west.yml`:

```yaml
manifest:
  projects:
    - name: frtc8900-zephyr
      url: https://github.com/sahko123/frtc8900-zephyr
      revision: main
      path: modules/rtc/frtc8900
```

Then run `west update`.

## Kconfig

| Symbol | Default | Description |
|---|---|---|
| `CONFIG_RTC_FRTC8900` | `y` when DT node present | Enable the driver |
| `CONFIG_RTC_ALARM` | selected automatically | Required for alarm support |

## Important hardware notes

### VLF flag
After a power loss (VDD dropped below ~1.6 V) or oscillator stop, the VLF flag
is set and **all registers must be reinitialised**. `rtc_get_time()` returns
`-ENODATA` when VLF is set. Calling `rtc_set_time()` with the correct time
automatically clears VLF; subsequent `rtc_get_time()` calls will succeed.

### Alarm AE bits (low-active)
The FRTC8900 alarm enable bits work inverted relative to most RTCs: `AE=0`
means the field **participates** in the alarm comparison, `AE=1` means it is
**ignored**. The driver handles this transparently via the standard Zephyr
alarm mask API.

### WEEK register
The day-of-week register is a bitmask (bit 0 = Sunday, bit 6 = Saturday), not
a sequential 0–6 integer. The driver converts to/from `tm_wday` automatically.

### MONTHDAY vs WEEKDAY alarm
Setting both `RTC_ALARM_TIME_MASK_MONTHDAY` and `RTC_ALARM_TIME_MASK_WEEKDAY`
in the same alarm call returns `-EINVAL` — the hardware supports only one mode
at a time, selected by the `WADA` bit in the extension register.

### /INT pin
The `/INT` pin is open-drain and shared by alarm, timer, and time-update
interrupt sources. The driver reads the flag register in the work handler to
identify the source. A pull-up resistor is required on the PCB (or enabled via
`GPIO_PULL_UP` in the devicetree).

## License

Apache-2.0
