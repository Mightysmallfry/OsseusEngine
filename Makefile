.PHONY: configure build sandbox test clean rebuild


BUILD_DIR := build


configure:
	cmake --preset default

build: configure
	cmake --build $(BUILD_DIR) -j$(shell nproc)

sandbox: configure
	cmake --build $(BUILD_DIR) --target osseus-sandbox
	
run-sandbox: sandbox
	$(BUILD_DIR)/Sandbox/osseus-sandbox

test: build
	ctest --test-dir $(BUILD_DIR) --output-on-failure

clean:
	rm -rf $(BUILD_DIR)

rebuild: clean configure build