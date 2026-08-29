.PHONY: configure build osseus sandbox run-sandbox test clean rebuild


BUILD_DIR := build
JOBS := $(shell nproc)


configure:
	cmake --preset default

build: configure
	cmake --build $(BUILD_DIR) -j$(JOBS)

osseus: configure
	cmake --build $(BUILD_DIR) --target Osseus -j$(JOBS)

sandbox: configure
	cmake --build $(BUILD_DIR) --target osseus-sandbox -j$(JOBS)

run-sandbox: sandbox
	$(BUILD_DIR)/Sandbox/osseus-sandbox

test: build
	ctest --test-dir $(BUILD_DIR) --output-on-failure

clean:
	rm -rf $(BUILD_DIR)

rebuild: clean build