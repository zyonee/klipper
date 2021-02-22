# Support for ST7920 (128x64 graphics) LCD displays
#
# Copyright (C) 2018  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
from . import font8x14

BACKGROUND_PRIORITY_CLOCK = 0x7fffffff00000000

# Spec says 72us, but faster is possible in practice
ST7920_CMD_DELAY  = .000020
ST7920_SYNC_DELAY = .000045

TextGlyphs = { 'right_arrow': '\x1a' }
CharGlyphs = { 'degrees': bytearray(font8x14.VGA_FONT[0xf8]) }

class DisplayBase:
    def __init__(self):
        # framebuffers
        self.text_framebuffer = bytearray(' '*64)
        self.glyph_framebuffer = bytearray(128)
        self.graphics_framebuffers = [bytearray(32) for i in range(32)]
        self.all_framebuffers = [
            # Text framebuffer
            (self.text_framebuffer, bytearray('~'*64), 0x80),
            # Glyph framebuffer
            (self.glyph_framebuffer, bytearray('~'*128), 0x40),
            # Graphics framebuffers
            ] + [(self.graphics_framebuffers[i], bytearray('~'*32), i)
                 for i in range(32)]
        self.cached_glyphs = {}
        self.icons = {}
    def flush(self):
        # Find all differences in the framebuffers and send them to the chip
        for new_data, old_data, fb_id in self.all_framebuffers:
            if new_data == old_data:
                continue
            # Find the position of all changed bytes in this framebuffer
            diffs = [[i, 1] for i, (n, o) in enumerate(zip(new_data, old_data))
                     if n != o]
            # Batch together changes that are close to each other
            for i in range(len(diffs)-2, -1, -1):
                pos, count = diffs[i]
                nextpos, nextcount = diffs[i+1]
                if pos + 5 >= nextpos and nextcount < 16:
                    diffs[i][1] = nextcount + (nextpos - pos)
                    del diffs[i+1]
            # Transmit changes
            for pos, count in diffs:
                count += pos & 0x01
                count += count & 0x01
                pos = pos & ~0x01
                chip_pos = pos >> 1
                if fb_id < 0x40:
                    # Graphics framebuffer update
                    self.send([0x80 + fb_id, 0x80 + chip_pos], is_extended=True)
                else:
                    self.send([fb_id + chip_pos])
                self.send(new_data[pos:pos+count], is_data=True)
            old_data[:] = new_data
    def init(self):
        cmds = [0x24, # Enter extended mode
                0x40, # Clear vertical scroll address
                0x02, # Enable CGRAM access
                0x26, # Enable graphics
                0x22, # Leave extended mode
                0x02, # Home the display
                0x06, # Set positive update direction
                0x0c] # Enable display and hide cursor
        self.send(cmds)
        self.flush()
    def cache_glyph(self, glyph_name, base_glyph_name, glyph_id):
        icon = self.icons.get(glyph_name)
        base_icon = self.icons.get(base_glyph_name)
        if icon is None or base_icon is None:
            return
        all_bits = zip(icon[0], icon[1], base_icon[0], base_icon[1])
        for i, (ic1, ic2, b1, b2) in enumerate(all_bits):
            x1, x2 = ic1 ^ b1, ic2 ^ b2
            pos = glyph_id*32 + i*2
            self.glyph_framebuffer[pos:pos+2] = [x1, x2]
            self.all_framebuffers[1][1][pos:pos+2] = [x1 ^ 1, x2 ^ 1]
        self.cached_glyphs[glyph_name] = (base_glyph_name, (0, glyph_id*2))
    def set_glyphs(self, glyphs):
        for glyph_name, glyph_data in glyphs.items():
            icon = glyph_data.get('icon16x16')
            if icon is not None:
                self.icons[glyph_name] = icon
        # Setup animated glyphs
        self.cache_glyph('fan2', 'fan1', 0)
        self.cache_glyph('bed_heat2', 'bed_heat1', 1)
    def write_text(self, x, y, data):
        if x + len(data) > 16:
            data = data[:16 - min(x, 16)]
        pos = [0, 32, 16, 48][y] + x
        self.text_framebuffer[pos:pos+len(data)] = data
    def write_graphics(self, x, y, data):
        if x >= 16 or y >= 4 or len(data) != 16:
            return
        gfx_fb = y * 16
        if gfx_fb >= 32:
            gfx_fb -= 32
            x += 16
        for i, bits in enumerate(data):
            self.graphics_framebuffers[gfx_fb + i][x] = bits
    def write_glyph(self, x, y, glyph_name):
        glyph_id = self.cached_glyphs.get(glyph_name)
        if glyph_id is not None and x & 1 == 0:
            # Render cached icon using character generator
            glyph_name = glyph_id[0]
            self.write_text(x, y, glyph_id[1])
        icon = self.icons.get(glyph_name)
        if icon is not None:
            # Draw icon in graphics mode
            self.write_graphics(x, y, icon[0])
            self.write_graphics(x + 1, y, icon[1])
            return 2
        char = TextGlyphs.get(glyph_name)
        if char is not None:
            # Draw character
            self.write_text(x, y, char)
            return 1
        font = CharGlyphs.get(glyph_name)
        if font is not None:
            # Draw single width character
            self.write_graphics(x, y, font)
            return 1
        return 0
    def clear(self):
        self.text_framebuffer[:] = ' '*64
        zeros = bytearray(32)
        for gfb in self.graphics_framebuffers:
            gfb[:] = zeros
    def get_dimensions(self):
        return (16, 4)

# Display driver for stock ST7920 displays
class ST7920(DisplayBase):
    def __init__(self, config):
        printer = config.get_printer()
        # pin config
        ppins = printer.lookup_object('pins')
        pins = [ppins.lookup_pin(config.get(name + '_pin'))
                for name in ['cs', 'sclk', 'sid']]
        mcu = None
        for pin_params in pins:
            if mcu is not None and pin_params['chip'] != mcu:
                raise ppins.error("st7920 all pins must be on same mcu")
            mcu = pin_params['chip']
        self.pins = [pin_params['pin'] for pin_params in pins]
        # prepare send functions
        self.mcu = mcu
        self.oid = self.mcu.create_oid()
        self.mcu.register_config_callback(self.build_config)
        self.send_data_cmd = self.send_cmds_cmd = None
        self.is_extended = False
        # init display base
        DisplayBase.__init__(self)
    def build_config(self):
        # configure send functions
        self.mcu.add_config_cmd(
            "config_st7920 oid=%u cs_pin=%s sclk_pin=%s sid_pin=%s"
            " sync_delay_ticks=%d cmd_delay_ticks=%d" % (
                self.oid, self.pins[0], self.pins[1], self.pins[2],
                self.mcu.seconds_to_clock(ST7920_SYNC_DELAY),
                self.mcu.seconds_to_clock(ST7920_CMD_DELAY)))
        cmd_queue = self.mcu.alloc_command_queue()
        self.send_cmds_cmd = self.mcu.lookup_command(
            "st7920_send_cmds oid=%c cmds=%*s", cq=cmd_queue)
        self.send_data_cmd = self.mcu.lookup_command(
            "st7920_send_data oid=%c data=%*s", cq=cmd_queue)
    def send(self, cmds, is_data=False, is_extended=False):
        cmd_type = self.send_cmds_cmd
        if is_data:
            cmd_type = self.send_data_cmd
        elif self.is_extended != is_extended:
            add_cmd = 0x22
            if is_extended:
                add_cmd = 0x26
            cmds = [add_cmd] + cmds
            self.is_extended = is_extended
        cmd_type.send([self.oid, cmds], reqclock=BACKGROUND_PRIORITY_CLOCK)
        #logging.debug("st7920 %d %s", is_data, repr(cmds))

# Display driver for displays that emulate the ST7920 in software.
# These displays rely on the CS pin to be toggled in order to initialize the
# SPI correctly. This display driver uses a software SPI with an unused pin
# as the MISO pin.
class ST7920E(DisplayBase):
    def __init__(self, config):
        printer = config.get_printer()
        # pin config
        ppins = printer.lookup_object('pins')
        pins = [ppins.lookup_pin(config.get(name + '_pin'))
                for name in ['cs', 'sclk', 'sid', 'dummy']]
        mcu = None
        for pin_params in pins:
            if mcu is not None and pin_params['chip'] != mcu:
                raise ppins.error("st7920 all pins must be on same mcu")
            mcu = pin_params['chip']
        self.pins = [pin_params['pin'] for pin_params in pins]
        # prepare send function and cs pin
        self.mcu = mcu
        self.oid_spi = self.mcu.create_oid()
        self.oid_cs = self.mcu.create_oid()
        self.cmd_queue = self.mcu.alloc_command_queue()
        self.mcu.add_config_cmd("config_spi_without_cs oid=%d"
                           % (self.oid_spi,))
        self.mcu.add_config_cmd("config_digital_out oid=%d pin=%s value=%d"
                           " default_value=%d max_duration=%d"
                           % (self.oid_cs, self.pins[0], 0, 0, 0))
        self.mcu.register_config_callback(self.build_config)
        self.spi_send = self.cs = None
        self.is_extended = False
        # init display base
        DisplayBase.__init__(self)
    def build_config(self):
        # configure software spi
        self.mcu.add_config_cmd("spi_set_software_bus oid=%d"
            " miso_pin=%s mosi_pin=%s sclk_pin=%s mode=%d rate=%d"
            % (self.oid_spi, self.pins[3], self.pins[2], self.pins[1],
                0, 1000000))
        self.spi_send = self.mcu.lookup_command(
            "spi_send oid=%c data=%*s", cq=self.cmd_queue)
        # configure cs pin
        self.cs = self.mcu.lookup_command(
            "update_digital_out oid=%c value=%c", cq=self.cmd_queue)
    def send(self, cmds, is_data=False, is_extended=False):
        # setup sync byte and check for exten mode switch
        sync_byte = 0xfa
        if not is_data:
            sync_byte = 0xf8
            if self.is_extended != is_extended:
                add_cmd = 0x22
                if is_extended:
                    add_cmd = 0x26
                cmds = [add_cmd] + cmds
                self.is_extended = is_extended
        # copy data to ST7920 data format
        spi_data = [0] * (2 * len(cmds) + 1)
        spi_data[0] = sync_byte
        i = 1
        for b in cmds:
            spi_data[i] = b & 0xF0
            spi_data[i + 1] = (b & 0x0F) << 4
            i = i + 2
        # send data
        self.setCs(1)
        self.spi_send.send([self.oid_spi, spi_data],
                               minclock=0, reqclock=BACKGROUND_PRIORITY_CLOCK)
        self.setCs(0)
        #logging.debug("st7920 %d %s", is_data, repr(spi_data))
    def setCs(self, value):
        self.cs.send([self.oid_cs, not not value],
            minclock=0, reqclock=BACKGROUND_PRIORITY_CLOCK)
