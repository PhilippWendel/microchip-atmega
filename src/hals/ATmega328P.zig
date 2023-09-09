const std = @import("std");
const micro = @import("microzig");

const hw = micro.core.experimental;
const peripherals = micro.chip.peripherals;
const USART0 = peripherals.USART0;
const I2C = peripherals.TWI;
const CPU = peripherals.CPU;
const PRR = CPU.PRR;

pub const cpu = micro.cpu;
const Port = enum(u8) {
    B = 1,
    C = 2,
    D = 3,
};

pub const clock = struct {
    pub const Domain = enum {
        cpu,
    };
};

pub fn parse_pin(comptime spec: []const u8) type {
    const invalid_format_msg = "The given pin '" ++ spec ++ "' has an invalid format. Pins must follow the format \"P{Port}{Pin}\" scheme.";

    if (spec.len != 3)
        @compileError(invalid_format_msg);
    if (spec[0] != 'P')
        @compileError(invalid_format_msg);

    return struct {
        pub const port: Port = std.meta.stringToEnum(Port, spec[1..2]) orelse @compileError(invalid_format_msg);
        pub const pin: u3 = std.fmt.parseInt(u3, spec[2..3], 10) catch @compileError(invalid_format_msg);
    };
}

pub const gpio = struct {
    fn regs(comptime desc: type) type {
        return struct {
            // io address
            const pin_addr: u5 = 3 * @intFromEnum(desc.port) + 0x00;
            const dir_addr: u5 = 3 * @intFromEnum(desc.port) + 0x01;
            const port_addr: u5 = 3 * @intFromEnum(desc.port) + 0x02;

            // ram mapping
            const pin = @as(*volatile u8, @ptrFromInt(0x20 + @as(usize, pin_addr)));
            const dir = @as(*volatile u8, @ptrFromInt(0x20 + @as(usize, dir_addr)));
            const port = @as(*volatile u8, @ptrFromInt(0x20 + @as(usize, port_addr)));
        };
    }

    pub fn setOutput(comptime pin: type) void {
        cpu.sbi(regs(pin).dir_addr, pin.pin);
    }

    pub fn setInput(comptime pin: type) void {
        cpu.cbi(regs(pin).dir_addr, pin.pin);
    }

    pub fn read(comptime pin: type) hw.gpio.State {
        return if ((regs(pin).pin.* & (1 << pin.pin)) != 0)
            .high
        else
            .low;
    }

    pub fn write(comptime pin: type, state: hw.gpio.State) void {
        if (state == .high) {
            cpu.sbi(regs(pin).port_addr, pin.pin);
        } else {
            cpu.cbi(regs(pin).port_addr, pin.pin);
        }
    }

    pub fn toggle(comptime pin: type) void {
        cpu.sbi(regs(pin).pin_addr, pin.pin);
    }
};

pub const uart = struct {
    pub const DataBits = enum {
        five,
        six,
        seven,
        eight,
        nine,
    };

    pub const StopBits = enum {
        one,
        two,
    };

    pub const Parity = enum {
        odd,
        even,
    };
};

pub fn Uart(comptime index: usize, comptime pins: hw.uart.Pins) type {
    if (index != 0) @compileError("Atmega328p only has a single uart!");
    if (pins.tx != null or pins.rx != null)
        @compileError("Atmega328p has fixed pins for uart!");

    return struct {
        const Self = @This();

        fn computeDivider(baud_rate: u32) !u12 {
            const pclk = hw.clock.get().cpu;
            const divider = ((pclk + (8 * baud_rate)) / (16 * baud_rate)) - 1;

            return std.math.cast(u12, divider) orelse return error.UnsupportedBaudRate;
        }

        fn computeBaudRate(divider: u12) u32 {
            return hw.clock.get().cpu / (16 * @as(u32, divider) + 1);
        }

        pub fn init(config: hw.uart.Config) !Self {
            const ucsz: u3 = switch (config.data_bits) {
                .five => 0b000,
                .six => 0b001,
                .seven => 0b010,
                .eight => 0b011,
                .nine => return error.UnsupportedWordSize, // 0b111
            };

            const upm: u2 = if (config.parity) |parity| switch (parity) {
                .even => @as(u2, 0b10), // even
                .odd => @as(u2, 0b11), // odd
            } else 0b00; // parity disabled

            const usbs: u1 = switch (config.stop_bits) {
                .one => 0b0,
                .two => 0b1,
            };

            const umsel: u2 = 0b00; // Asynchronous USART

            // baud is computed like this:
            //             f(osc)
            // BAUD = ----------------
            //        16 * (UBRRn + 1)

            const ubrr_val = try computeDivider(config.baud_rate);

            USART0.UCSR0A.modify(.{
                .MPCM0 = 0,
                .U2X0 = 0,
            });
            USART0.UCSR0B.write(.{
                .TXB80 = 0, // we don't care about these btw
                .RXB80 = 0, // we don't care about these btw
                .UCSZ02 = @as(u1, @truncate((ucsz & 0x04) >> 2)),
                .TXEN0 = 1,
                .RXEN0 = 1,
                .UDRIE0 = 0, // no interrupts
                .TXCIE0 = 0, // no interrupts
                .RXCIE0 = 0, // no interrupts
            });
            USART0.UCSR0C.write(.{
                .UCPOL0 = 0, // async mode
                .UCSZ0 = @as(u2, @truncate((ucsz & 0x03) >> 0)),
                .USBS0 = .{ .raw = usbs },
                .UPM0 = .{ .raw = upm },
                .UMSEL0 = .{ .raw = umsel },
            });

            USART0.UBRR0 = ubrr_val;

            return Self{};
        }

        pub fn canWrite(self: Self) bool {
            _ = self;
            return (USART0.UCSR0A.read().UDRE0 == 1);
        }

        pub fn tx(self: Self, ch: u8) void {
            while (!self.canWrite()) {} // Wait for Previous transmission
            USART0.UDR0 = ch; // Load the data to be transmitted
        }

        pub fn canRead(self: Self) bool {
            _ = self;
            return (USART0.UCSR0A.read().RXC0 == 1);
        }

        pub fn rx(self: Self) u8 {
            while (!self.canRead()) {} // Wait till the data is received
            return USART0.UDR0; // Read received data
        }
    };
}

pub fn I2CController(comptime index: usize, comptime pins: hw.i2c.Pins) type {
    // - Terminology: I2C instead of TWI (TwoWireInterface) and controller/device, instead of master/slave
    // - TODO
    //   - Rename variables and functions to match zig naming conventions
    //   - Add status codes
    //   - Init
    //     - internal pullups
    //       - add to config
    //       - Maybe convert to use gpio instead of writing to the port directly
    //     - Check if bitrate and prescaler calculations are correct
    //     - Check if bitrate >= 10 applies
    //   - WriteState
    //     - Figure out what to do with reload and buffer
    //   - ReadState
    //     - start
    //     - read_no_eof
    //     - stop
    //     - restart_read
    //     - restart_write
    if (index != 0) @compileError("Index is not supported");
    if (pins.scl != null or pins.sda != null) @compileError("Custom pins are not supported");

    const bitrate_register = I2C.TWBR;
    const control_register = I2C.TWCR;
    const status_register = I2C.TWSR;
    const data_register = I2C.TWDR;
    // Currently unused
    // const address_register = I2C.TWAR;

    const Prescalers = I2C.COMM_TWI_PRESACLE;

    return struct {
        const Self = @This();

        pub fn init(config: hw.i2c.Config) !Self {
            // "Up to 400kHz data transfer speed" [1, p. 173]
            if (config.target_speed > 400_000)
                @compileError("config.target_speed can't be greater than 400kHz");
            // "[Device] operation does not depend on bit rate or prescaler settings,
            // but the CPU clock frequency in the [device] must be at least 16 times higher than the SCL frequency." [1, p. 180]
            if (!(hw.clock.get().cpu > config.target_speed * 16))
                @compileError("CPU clock frequency must be at least 16 times higher than config.target_speed");

            // "[I]nternal pull-ups in the AVR pads can be enabled by setting the PORT bits corresponding to the SCL and SDA pins" [1, p. 179]
            const enable_internal_pullups = true;
            if (enable_internal_pullups) {
                peripherals.PORT.PORTC.PORTC |= 0b0011_0000; // SDA = PC4, SCL = PC5
            }

            PRR.modify(.{ .PRTWI = 0 }); // "The PRTWI bit ... must be written to zero to enable the 2-wire serial interface." [1, p. 174]

            // SCL_frequency = CPU_Clock_frequency/(16+2(TWBR)*(PrescalerValue)) [1, p. 180]
            const @"bitrate*prescaler" = ((hw.clock.get().cpu / config.target_speed) - 16) / 2;
            const bitrate_and_prescaler = inline for (Prescalers) |prescaler| {
                const bitrate = @"bitrate*prescaler" / std.parseUnsigned(u8, @typeName(prescaler), 0);
                // Bitrate should be at least 10 [2, Sec. Note 5]
                return if (10 <= bitrate and bitrate <= std.math.maxInt(@TypeOf(bitrate_register))) .{ .bitrate = bitrate, .prescaler = prescaler };
            } else @compileError("I2C: bitrate and prescalar could not be calculated");

            bitrate_register = bitrate_and_prescaler.bitrate;
            status_register.modify(.{ .TWPS = bitrate_and_prescaler.prescaler });

            control_register.modify(.{ .TWEN = 1 });

            return Self{};
        }

        pub const WriteState = struct {
            address: u7,
            buffer: [255]u8 = undefined,
            buffer_size: u8 = 0,

            pub fn start(address: u7) !WriteState {
                // Steps 1 - 3 [1, p. 182] and example code at [1, p. 183]
                control_register.modify(.{ .TWSTA = 1, .TWEN = 1, .TWINT = 1 });
                wait_for_i2c_to_finish_operation();
                if (status_register.read().TWS & 0x08) // Code from [1, p. 186]
                    return 0; // TODO Here we should return an error
                return WriteState{ .address = address };
            }

            pub fn write_all(self: *WriteState, bytes: []const u8) !void {
                std.debug.assert(self.buffer_size < 255);
                for (bytes) |b| {
                    self.buffer[self.buffer_size] = b;
                    self.buffer_size += 1;
                    if (self.buffer_size == 255) {
                        try self.send_buffer(1);
                    }
                }
            }

            fn send_buffer(self: *WriteState, reload: u1) !void {
                if (self.buffer_size == 0) @panic("write of 0 bytes not supported");

                std.debug.assert(reload == 0 or self.buffer_size == 255); // see TODOs below

                // Send address
                try transmit(self.address);

                // Send data
                for (self.buffer[0..self.buffer_size]) |b| {
                    try transmit(b);
                }
                self.buffer_size = 0;
            }

            pub fn stop(self: *WriteState) !void {
                _ = self;
                // example code at [1, p. 183]
                control_register.modify(.{ .TWINT = 1, .TWEN = 1, .TWSTO = 1 });
            }

            pub fn restart_read(self: *WriteState) !ReadState {
                try self.send_buffer(0);
                return ReadState{ .address = self.address };
            }
            pub fn restart_write(self: *WriteState) !WriteState {
                try self.send_buffer(0);
                return WriteState{ .address = self.address };
            }

            fn transmit(data: u8) !void {
                // example code at [1, p. 183]
                data_register = data;
                control_register.modify(.{ .TWINT = 1, .TWEN = 1 });
                wait_for_i2c_to_finish_operation();
                if (status_register.read().TWS == 0) // TODO Check real status codes
                    return 0;
            }
            inline fn wait_for_i2c_to_finish_operation() void {
                // "When the [I2C] has finished an operation and expects application response, the TWINT flag is set." [1, p.182]
                while (!control_register.read().TWINT) {}
            }
        };

        pub const ReadState = struct {
            address: u7,
            read_allowed: bool = true,

            pub fn start(address: u7) !ReadState {
                return ReadState{ .address = address };
            }

            /// Fails with ReadError if incorrect number of bytes is received.
            pub fn read_no_eof(self: *ReadState, buffer: []u8) !void {
                _ = self;
                _ = buffer;
            }

            pub fn stop(self: *ReadState) !void {
                _ = self;
            }

            pub fn restart_read(self: *ReadState) !ReadState {
                return ReadState{ .address = self.address };
            }
            pub fn restart_write(self: *ReadState) !WriteState {
                return WriteState{ .address = self.address };
            }
        };
    };
}

// References:
// [1] https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-7810-Automotive-Microcontrollers-ATmega328P_Datasheet.pdf
// [2] https://www.nongnu.org/avr-libc/user-manual/group__twi__demo.html
