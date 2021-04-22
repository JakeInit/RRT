all:
	@mkdir -p .build
	-mkdir -p bin
	-mkdir -p lib
	cd .build && cmake ..
	cd .build && $(MAKE)

clean:
	cd .build && $(MAKE) clean
	rm -rf .build
