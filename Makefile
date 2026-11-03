# AGC Lander: Luminary099 running inside a raylib game.
#   make              Linux build (build/linux/)
#   make windows      Windows build via mingw-w64 (build/win/)
#   make test         headless AGC self-test on Linux
#   make test-wine    headless AGC self-test under Wine

VAGC    := third_party/virtualagc
RAYLIB  := third_party/raylib/src
ROPESRC := $(VAGC)/Luminary099
YAYUL   := build/tools/yaYUL
ROPE    := build/Luminary099.bin
ROPE_C  := build/rope_luminary099.c
SYMS_C  := build/agc_symbols.c
FONTS_C := build/font_jbm.c build/font_rc.c build/font_rcb.c

AGC_CORE := $(VAGC)/yaAGC/agc_engine.c $(VAGC)/yaAGC/agc_engine_init.c $(VAGC)/yaAGC/agc_utilities.c
HOST     := src/agc_host.c src/agc_mem.c src/moon.c src/lm.c src/coop.c src/net.c $(ROPE_C) $(SYMS_C)
GAME     := src/main.c src/dsky.c src/scenario.c src/crew.c src/hints.c src/terrain.c src/render.c src/lmmodel.c src/hud.c src/cockpit.c src/cockpit_panels.c src/sky_extras.c src/controls.c src/rocks.c data/a11_pdi_lgc.c $(FONTS_C)

# Vendor core: build quietly, it predates modern warning sets.
CORE_FLAGS := -O2 -w -I$(VAGC)/yaAGC
APP_FLAGS  := -O2 -Wall -Wextra -std=gnu11 -Isrc -I$(VAGC)/yaAGC -I$(RAYLIB)

LINUX_CC   := gcc
WIN_CC     := x86_64-w64-mingw32-gcc
WIN_AR     := x86_64-w64-mingw32-ar

.PHONY: all linux windows test test-wine rope clean distclean install uninstall dist

all: linux

# ---- Rope: assemble Luminary099 with yaYUL and verify against the real rope dump.
$(YAYUL): $(wildcard $(VAGC)/yaYUL/*.c $(VAGC)/yaYUL/*.h)
	@mkdir -p $(dir $@)
	$(LINUX_CC) -O2 -w -DNVER='"\"agc-lander\""' -o $@ $(VAGC)/yaYUL/*.c -lm

$(ROPE): $(YAYUL) $(wildcard $(ROPESRC)/*.agc)
	@mkdir -p build/rope-work
	cp $(ROPESRC)/*.agc build/rope-work/
	cd build/rope-work && ../../$(YAYUL) --unpound-page MAIN.agc > ../yaYUL-Luminary099.lst 2>&1
	cp build/rope-work/MAIN.agc.bin $@
	python3 -I tools/check_rope.py $(ROPESRC)/AP11ROPE.binsource $@

$(ROPE_C): $(ROPE) tools/embed_rope.py
	python3 -I tools/embed_rope.py $< $@

$(SYMS_C): $(ROPE) tools/gen_symbols.py
	python3 -I tools/gen_symbols.py build/yaYUL-Luminary099.lst $@

build/font_jbm.c: data/fonts/JetBrainsMono-Regular.ttf tools/embed_bin.py
	@mkdir -p $(dir $@)
	python3 -I tools/embed_bin.py $< font_jbm $@
build/font_rc.c: data/fonts/RobotoCondensed-Regular.ttf tools/embed_bin.py
	@mkdir -p $(dir $@)
	python3 -I tools/embed_bin.py $< font_rc $@
build/font_rcb.c: data/fonts/RobotoCondensed-Bold.ttf tools/embed_bin.py
	@mkdir -p $(dir $@)
	python3 -I tools/embed_bin.py $< font_rcb $@

rope: $(ROPE)

# ---- raylib static libraries.
build/raylib-linux/libraylib.a:
	@mkdir -p $(dir $@)
	$(MAKE) -C $(RAYLIB) clean
	$(MAKE) -C $(RAYLIB) PLATFORM=PLATFORM_DESKTOP RAYLIB_RELEASE_PATH=$(abspath $(dir $@))
	$(MAKE) -C $(RAYLIB) clean

build/raylib-win/libraylib.a:
	@mkdir -p $(dir $@)
	$(MAKE) -C $(RAYLIB) clean
	$(MAKE) -C $(RAYLIB) PLATFORM=PLATFORM_DESKTOP OS=Windows_NT CC=$(WIN_CC) AR=$(WIN_AR) RAYLIB_RELEASE_PATH=$(abspath $(dir $@))
	$(MAKE) -C $(RAYLIB) clean

# ---- Linux.
build/linux/core.a: $(AGC_CORE)
	@mkdir -p build/linux/core
	cd build/linux/core && $(LINUX_CC) $(CORE_FLAGS:-I%=-I../../../%) -c $(addprefix ../../../,$(AGC_CORE))
	ar rcs $@ build/linux/core/*.o

linux: build/linux/agc-lander build/linux/agc-selftest build/linux/agc-lab

build/linux/agc-lab: src/lab.c src/scenario.c src/crew.c data/a11_pdi_lgc.c $(HOST) src/*.h build/linux/core.a
	$(LINUX_CC) $(APP_FLAGS) -o $@ src/lab.c src/scenario.c src/crew.c data/a11_pdi_lgc.c $(HOST) build/linux/core.a -lm

build/linux/agc-lander: $(GAME) $(HOST) src/*.h build/linux/core.a build/raylib-linux/libraylib.a
	$(LINUX_CC) $(APP_FLAGS) -o $@ $(GAME) $(HOST) build/linux/core.a build/raylib-linux/libraylib.a -lm -lpthread -ldl

build/linux/agc-selftest: src/selftest.c $(HOST) src/agc_host.h build/linux/core.a
	$(LINUX_CC) $(APP_FLAGS) -o $@ src/selftest.c $(HOST) build/linux/core.a -lm

test: build/linux/agc-selftest build/linux/agc-lab
	./build/linux/agc-selftest
	./tools/test_landing.sh build/linux/agc-lab

# ---- Windows (single static .exe).
build/win/core.a: $(AGC_CORE)
	@mkdir -p build/win/core
	cd build/win/core && $(WIN_CC) -DWIN32 $(CORE_FLAGS:-I%=-I../../../%) -c $(addprefix ../../../,$(AGC_CORE))
	$(WIN_AR) rcs $@ build/win/core/*.o

WIN_LINK := -static -lopengl32 -lgdi32 -lwinmm -lws2_32 -lpthread

windows: build/win/agc-lander.exe build/win/agc-selftest.exe

build/win/agc-lander.exe: $(GAME) $(HOST) src/*.h build/win/core.a build/raylib-win/libraylib.a
	$(WIN_CC) $(APP_FLAGS) -DWIN32 -mwindows -o $@ $(GAME) $(HOST) build/win/core.a build/raylib-win/libraylib.a $(WIN_LINK)

build/win/agc-selftest.exe: src/selftest.c $(HOST) src/agc_host.h build/win/core.a
	$(WIN_CC) $(APP_FLAGS) -DWIN32 -o $@ src/selftest.c $(HOST) build/win/core.a -static -lws2_32

test-wine: build/win/agc-selftest.exe
	WINEDEBUG=-all wine ./build/win/agc-selftest.exe

clean:
	rm -rf build/linux build/win build/rope-work $(ROPE) $(ROPE_C)

distclean:
	rm -rf build

# ---- Linux install (Omarchy / any freedesktop launcher), per user.
PREFIX ?= $(HOME)/.local
install: build/linux/agc-lander
	install -Dm755 build/linux/agc-lander $(PREFIX)/bin/agc-lander
	install -Dm644 data/icon/agc-lander.png $(PREFIX)/share/icons/hicolor/512x512/apps/agc-lander.png
	install -Dm644 data/agc-lander.desktop $(PREFIX)/share/applications/agc-lander.desktop
	-update-desktop-database $(PREFIX)/share/applications 2>/dev/null
	-gtk-update-icon-cache -q $(PREFIX)/share/icons/hicolor 2>/dev/null

uninstall:
	rm -f $(PREFIX)/bin/agc-lander $(PREFIX)/share/icons/hicolor/512x512/apps/agc-lander.png $(PREFIX)/share/applications/agc-lander.desktop

# ---- Release archives.
dist: build/linux/agc-lander build/win/agc-lander.exe
	rm -rf build/dist && mkdir -p build/dist/agc-lander-linux build/dist/agc-lander-windows
	cp build/linux/agc-lander README.md data/agc-lander.desktop data/icon/agc-lander.png build/dist/agc-lander-linux/
	printf '#!/bin/sh\n# Installs AGC Lander for this user (~/.local) with a launcher entry.\nset -e\ncd "$$(dirname "$$0")"\ninstall -Dm755 agc-lander "$$HOME/.local/bin/agc-lander"\ninstall -Dm644 agc-lander.png "$$HOME/.local/share/icons/hicolor/512x512/apps/agc-lander.png"\ninstall -Dm644 agc-lander.desktop "$$HOME/.local/share/applications/agc-lander.desktop"\necho "Installed: run agc-lander, or find AGC Lander in your app launcher."\n' > build/dist/agc-lander-linux/install.sh
	chmod +x build/dist/agc-lander-linux/install.sh
	cp build/win/agc-lander.exe README.md build/dist/agc-lander-windows/
	cd build/dist && tar czf agc-lander-linux-x86_64.tar.gz agc-lander-linux && zip -qr agc-lander-windows-x86_64.zip agc-lander-windows
