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
# Baked surface maps (tools/bake_textures.py), embedded in the PC executables.
TEX_NAMES := lm_panel lm_foil lm_blanket rock cabin cabin_wall regolith

AGC_CORE := $(VAGC)/yaAGC/agc_engine.c $(VAGC)/yaAGC/agc_engine_init.c $(VAGC)/yaAGC/agc_utilities.c
HOST     := src/agc_host.c src/agc_mem.c src/moon.c src/lm.c src/csm.c src/coop.c src/net.c $(ROPE_C) $(SYMS_C)
GAME     := src/main.c src/dsky.c src/scenario.c src/crew.c src/hints.c src/terrain.c src/render.c src/lmmodel.c src/hud.c src/cockpit.c src/cockpit_panels.c src/pcanvas.c src/sky_extras.c src/controls.c src/rocks.c src/assets.c src/sound.c src/eva.c src/ascent.c src/ascent_ui.c src/csmmodel.c data/a11_pdi_lgc.c data/a11_liftoff_lgc.c $(FONTS_C)

# Vendor core: build quietly, it predates modern warning sets.
CORE_FLAGS := -O2 -w -I$(VAGC)/yaAGC
APP_FLAGS  := -O2 -Wall -Wextra -std=gnu11 -Isrc -I$(VAGC)/yaAGC -I$(RAYLIB)

LINUX_CC   := gcc
WIN_CC     := x86_64-w64-mingw32-gcc
WIN_AR     := x86_64-w64-mingw32-ar

.PHONY: all linux windows test test-wine rope clean distclean install uninstall dist textures

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

# Surface maps: embedded as C arrays, compiled once per platform.
.PRECIOUS: build/tex_%.c
build/tex_%.c: data/textures/%.png tools/embed_bin.py
	@mkdir -p $(dir $@)
	python3 -I tools/embed_bin.py $< tex_$* $@
build/linux/tex_%.o: build/tex_%.c
	@mkdir -p $(dir $@)
	$(LINUX_CC) -c -o $@ $<
build/win/tex_%.o: build/tex_%.c
	@mkdir -p $(dir $@)
	$(WIN_CC) -c -o $@ $<
LINUX_TEX := $(TEX_NAMES:%=build/linux/tex_%.o)
WIN_TEX   := $(TEX_NAMES:%=build/win/tex_%.o)

# Re-bake the surface maps (needs numpy + Pillow); the results are committed.
textures:
	python3 tools/bake_textures.py data/textures

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

build/linux/agc-lab: src/lab.c src/scenario.c src/crew.c src/ascent.c data/a11_pdi_lgc.c data/a11_liftoff_lgc.c $(HOST) src/*.h build/linux/core.a
	$(LINUX_CC) $(APP_FLAGS) -o $@ src/lab.c src/scenario.c src/crew.c src/ascent.c data/a11_pdi_lgc.c data/a11_liftoff_lgc.c $(HOST) build/linux/core.a -lm

build/linux/agc-lander: $(GAME) $(HOST) src/*.h build/linux/core.a build/raylib-linux/libraylib.a $(LINUX_TEX)
	$(LINUX_CC) $(APP_FLAGS) -o $@ $(GAME) $(HOST) $(LINUX_TEX) build/linux/core.a build/raylib-linux/libraylib.a -lm -lpthread -ldl

build/linux/agc-selftest: src/selftest.c $(HOST) src/agc_host.h build/linux/core.a
	$(LINUX_CC) $(APP_FLAGS) -o $@ src/selftest.c $(HOST) build/linux/core.a -lm

test: build/linux/agc-selftest build/linux/agc-lab
	./build/linux/agc-selftest
	./tools/test_landing.sh build/linux/agc-lab
	./tools/test_ascent.sh build/linux/agc-lab
	./tools/test_docking.sh build/linux/agc-lab

# ---- Windows (single static .exe).
build/win/core.a: $(AGC_CORE)
	@mkdir -p build/win/core
	cd build/win/core && $(WIN_CC) -DWIN32 $(CORE_FLAGS:-I%=-I../../../%) -c $(addprefix ../../../,$(AGC_CORE))
	$(WIN_AR) rcs $@ build/win/core/*.o

WIN_LINK := -static -lopengl32 -lgdi32 -lwinmm -lws2_32 -lpthread

windows: build/win/agc-lander.exe build/win/agc-selftest.exe

build/win/agc-lander.exe: $(GAME) $(HOST) src/*.h build/win/core.a build/raylib-win/libraylib.a $(WIN_TEX)
	$(WIN_CC) $(APP_FLAGS) -DWIN32 -mwindows -o $@ $(GAME) $(HOST) $(WIN_TEX) build/win/core.a build/raylib-win/libraylib.a $(WIN_LINK)

build/win/agc-selftest.exe: src/selftest.c $(HOST) src/agc_host.h build/win/core.a
	$(WIN_CC) $(APP_FLAGS) -DWIN32 -o $@ src/selftest.c $(HOST) build/win/core.a -static -lws2_32

test-wine: build/win/agc-selftest.exe
	WINEDEBUG=-all wine ./build/win/agc-selftest.exe

# ---- Meta Quest (Android arm64, OpenXR, OpenGL ES 3). One APK, no Java code.
#   make quest           build/quest/agc-lander-quest.apk
#   make quest-install   install it on the headset over USB (developer mode, USB debugging allowed)
#   make vrsim           PC stand-in for the headset: same shaders and stereo path, two eyes in a window
ANDROID_SDK ?= $(HOME)/Android/Sdk
ANDROID_NDK ?= $(lastword $(sort $(wildcard $(ANDROID_SDK)/ndk/*)))
BUILD_TOOLS ?= $(lastword $(sort $(wildcard $(ANDROID_SDK)/build-tools/*)))
ANDROID_JAR ?= $(lastword $(sort $(wildcard $(ANDROID_SDK)/platforms/*/android.jar)))
QUEST_API   := 29
NDK_BIN     := $(ANDROID_NDK)/toolchains/llvm/prebuilt/linux-x86_64/bin
QUEST_CC    := $(NDK_BIN)/aarch64-linux-android$(QUEST_API)-clang
OPENXR_VER  := 1.1.63
OPENXR_SHA1 := bb72fea1e7bdd35b381ae2a705206d912a1d5b30
OPENXR_DIR  := build/quest/openxr
OPENXR_LIB  := $(OPENXR_DIR)/jni/arm64-v8a/libopenxr_loader.so
QUEST_KEY   ?= $(HOME)/.android/debug.keystore
QUEST_APK   := build/quest/agc-lander-quest.apk
QUEST_SO    := build/quest/apk/lib/arm64-v8a/libagclander.so
VR_COMMON   := $(filter-out src/controls.c,$(GAME)) src/controls_vr.c src/input.c src/gles_shim.c
QUEST_TEX   := $(TEX_NAMES:%=build/quest/tex_%.o)
QUEST_FLAGS := -O2 -fPIC -std=gnu11 -DAGC_GLES -DAGC_VR -DPLATFORM_ANDROID -Wall -Wextra -Wno-unused-function -Wno-unused-parameter \
               -Wno-unused-variable -Wno-unused-but-set-variable -Wno-missing-field-initializers \
               -Isrc -I$(VAGC)/yaAGC -I$(RAYLIB) \
               -I$(ANDROID_NDK)/sources/android/native_app_glue -I$(OPENXR_DIR)/prefab/modules/headers/include

.PHONY: quest quest-install vrsim

# Khronos OpenXR loader for Android (Apache-2.0 / MIT), from Maven Central.
$(OPENXR_LIB):
	@mkdir -p build/quest/dl
	curl -sSL -o build/quest/dl/openxr_loader.aar https://repo1.maven.org/maven2/org/khronos/openxr/openxr_loader_for_android/$(OPENXR_VER)/openxr_loader_for_android-$(OPENXR_VER).aar
	echo "$(OPENXR_SHA1)  build/quest/dl/openxr_loader.aar" | sha1sum -c -
	rm -rf $(OPENXR_DIR) && mkdir -p $(OPENXR_DIR)
	cd $(OPENXR_DIR) && unzip -q ../dl/openxr_loader.aar
	touch $@

# raylib for Android is compiled here directly: its own Makefile passes "-target aarch64",
# which makes current NDK compilers fall back to the PC's headers.
RAYLIB_QUEST_SRC := rcore rshapes rtextures rtext utils rmodels raudio
build/raylib-quest/libraylib.a: $(wildcard $(RAYLIB)/*.c $(RAYLIB)/*.h $(RAYLIB)/platforms/rcore_android.c)
	@mkdir -p $(dir $@)
	rm -f $(dir $@)*.o
	for f in $(RAYLIB_QUEST_SRC); do \
		$(QUEST_CC) -c -O2 -fPIC -w -std=gnu99 -fno-strict-aliasing -D_GNU_SOURCE -DPLATFORM_ANDROID -DGRAPHICS_API_OPENGL_ES3 \
			-I$(RAYLIB) -I$(ANDROID_NDK)/sources/android/native_app_glue -o $(dir $@)$$f.o $(RAYLIB)/$$f.c || exit 1; \
	done
	$(QUEST_CC) -c -O2 -fPIC -w -o $(dir $@)android_native_app_glue.o $(ANDROID_NDK)/sources/android/native_app_glue/android_native_app_glue.c
	$(NDK_BIN)/llvm-ar rcs $@ $(dir $@)*.o

build/quest/core.a: $(AGC_CORE)
	@mkdir -p build/quest/core
	cd build/quest/core && $(QUEST_CC) -fPIC $(CORE_FLAGS:-I%=-I../../../%) -c $(addprefix ../../../,$(AGC_CORE))
	$(NDK_BIN)/llvm-ar rcs $@ build/quest/core/*.o

build/quest/tex_%.o: build/tex_%.c
	@mkdir -p $(dir $@)
	$(QUEST_CC) -fPIC -c -o $@ $<

$(QUEST_SO): $(VR_COMMON) src/vr_openxr.c $(HOST) src/*.h build/quest/core.a build/raylib-quest/libraylib.a $(QUEST_TEX) $(OPENXR_LIB)
	@mkdir -p $(dir $@)
	$(QUEST_CC) $(QUEST_FLAGS) -shared -Wl,-soname,libagclander.so -u ANativeActivity_onCreate -Wl,--no-undefined -o $@ \
		$(VR_COMMON) src/vr_openxr.c $(HOST) $(QUEST_TEX) build/quest/core.a build/raylib-quest/libraylib.a \
		-L$(dir $(OPENXR_LIB)) -lopenxr_loader -llog -landroid -lEGL -lGLESv3 -lOpenSLES -lm -ldl
	$(NDK_BIN)/llvm-strip --strip-unneeded $@
	cp $(OPENXR_LIB) $(dir $@)

# A debug signing key (the standard Android one) if this machine has none yet.
$(QUEST_KEY):
	@mkdir -p $(dir $@)
	keytool -genkeypair -keystore $@ -storepass android -keypass android -alias androiddebugkey -keyalg RSA -keysize 2048 \
		-validity 10000 -dname "CN=Android Debug,O=Android,C=US"

$(QUEST_APK): $(QUEST_SO) quest/AndroidManifest.xml data/icon/agc-lander.png | $(QUEST_KEY)
	rm -rf build/quest/res build/quest/res.zip build/quest/base.apk build/quest/aligned.apk
	mkdir -p build/quest/res/mipmap-xxxhdpi
	cp data/icon/agc-lander.png build/quest/res/mipmap-xxxhdpi/ic_launcher.png
	$(BUILD_TOOLS)/aapt2 compile --dir build/quest/res -o build/quest/res.zip
	$(BUILD_TOOLS)/aapt2 link -o build/quest/base.apk --manifest quest/AndroidManifest.xml -I $(ANDROID_JAR) build/quest/res.zip
	cd build/quest/apk && zip -q -r ../base.apk lib
	$(BUILD_TOOLS)/zipalign -f -p 4 build/quest/base.apk build/quest/aligned.apk
	$(BUILD_TOOLS)/apksigner sign --ks $(QUEST_KEY) --ks-pass pass:android --key-pass pass:android --out $@ build/quest/aligned.apk
	@ls -la $@

quest: $(QUEST_APK)

# ADB_LIBUSB=1: the default adb server polls USB and makes audio on this PC stutter.
quest-install: $(QUEST_APK)
	ADB_LIBUSB=1 adb install -r $(QUEST_APK)

# PC stand-in for the headset (needs the system's EGL / GLESv2 libraries).
build/raylib-gles/libraylib.a:
	@mkdir -p $(dir $@)
	$(MAKE) -C $(RAYLIB) clean
	$(MAKE) -C $(RAYLIB) PLATFORM=PLATFORM_DESKTOP GRAPHICS=GRAPHICS_API_OPENGL_ES3 RAYLIB_RELEASE_PATH=$(abspath $(dir $@))
	$(MAKE) -C $(RAYLIB) clean

vrsim: build/gles/agc-lander-vrsim

build/gles/agc-lander-vrsim: $(VR_COMMON) src/vr_sim.c $(HOST) src/*.h build/linux/core.a build/raylib-gles/libraylib.a $(LINUX_TEX)
	@mkdir -p $(dir $@)
	$(LINUX_CC) $(APP_FLAGS) -DAGC_GLES -DAGC_VR -Wno-unused-function -Wno-unused-parameter -Wno-unused-variable -Wno-unused-but-set-variable \
		-o $@ $(VR_COMMON) src/vr_sim.c $(HOST) $(LINUX_TEX) build/linux/core.a build/raylib-gles/libraylib.a -lGLESv2 -lEGL -lm -lpthread -ldl

clean:
	rm -rf build/linux build/win build/gles build/quest/apk build/quest/core build/rope-work $(ROPE) $(ROPE_C)

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
