.PHONY: configure build test clean rebuild

BUILD_DIR := build

configure:
	cmake --preset default

build: configure
	cmake --build $(BUILD_DIR) -j$(shell nproc)

test: build
	ctest --test-dir $(BUILD_DIR) --output-on-failure

clean:
	rm -rf $(BUILD_DIR)

rebuild: clean configure build
