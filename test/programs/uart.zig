// Calls all the functions in the UART driver to make sure they compile.
const micro = @import("microzig");

pub fn main() !void {
    var uart = micro.core.experimental.uart.Uart(0, .{}).init(.{
        .baud_rate = 115200,
        .stop_bits = .one,
        .parity = null,
        .data_bits = .eight,
    }) catch unreachable;

    uart = micro.core.experimental.uart.Uart(0, .{}).get_or_init(.{
        .baud_rate = 115200,
        .stop_bits = .one,
        .parity = null,
        .data_bits = .eight,
    }) catch unreachable;

    _ = uart.can_read();
    _ = uart.can_write();

    try uart.writer().writeByte(0x00);
    _ = try uart.reader().readByte();

    while (true) {}
}
