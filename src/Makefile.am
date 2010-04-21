
if ARCH_LINUX
SUBDIRS = lib mos tools apps test
endif

if ARCH_MICRO
SUBDIRS = lib mos apps test
endif

.PHONY: $(APPS) mos

$(APPS):
	@echo "Making all in $@"
	@cd apps && make $@

mos:
	@echo "Making all in lib"
	@cd lib && make
if ARCH_LINUX
	@echo "Making all in tools"
	@cd tools && make
endif
	@echo "Making all in mos"
	@cd mos && make

