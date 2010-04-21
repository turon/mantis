
SUBDIRS = src 

.PHONY: $(APPS) mos

$(APPS): 
	@echo "Making all in src"
	@cd src && make mos && make $@

mos:
	@echo "Making mos in src"
	@cd src && make mos

all-local:
	@echo ""
	@echo "MOS Build complete"
	@echo ""
