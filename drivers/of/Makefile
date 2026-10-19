obj-y = base.o device.o platform.o
obj-$(CONFIG_OF_DYNAMIC) += dynamic.o
obj-$(CONFIG_OF_FLATTREE) += fdt.o
obj-$(CONFIG_OF_EARLY_FLATTREE) += fdt_address.o
obj-$(CONFIG_OF_EDITS) += fdt_edits.o fdt_edits_table.o
obj-$(CONFIG_OF_PROMTREE) += pdt.o
obj-$(CONFIG_OF_ADDRESS)  += address.o
obj-$(CONFIG_OF_IRQ)    += irq.o
obj-$(CONFIG_OF_NET)	+= of_net.o
obj-$(CONFIG_OF_UNITTEST) += unittest.o
obj-$(CONFIG_OF_MDIO)	+= of_mdio.o
obj-$(CONFIG_OF_PCI)	+= of_pci.o
obj-$(CONFIG_OF_PCI_IRQ)  += of_pci_irq.o
obj-$(CONFIG_OF_RESERVED_MEM) += of_reserved_mem.o
obj-$(CONFIG_OF_RESOLVE)  += resolver.o
obj-$(CONFIG_OF_OVERLAY) += overlay.o
obj-$(CONFIG_OF_NUMA) += of_numa.o

obj-$(CONFIG_OF_UNITTEST) += unittest-data/

# The generated table, from the build directory into drivers/of; copied again only when it changes.
ifdef CONFIG_OF_EDITS
ifeq ($(CONFIG_OF_EDITS_TABLE),"")
$(error CONFIG_OF_EDITS is set but CONFIG_OF_EDITS_TABLE is empty)
endif
of-edits-table := $(objtree)/$(patsubst "%",%,$(CONFIG_OF_EDITS_TABLE))
targets += fdt_edits_table.c
$(obj)/fdt_edits_table.c: $(of-edits-table) FORCE
	$(call if_changed,shipped)
$(of-edits-table):
	@echo >&2 "$@ (CONFIG_OF_EDITS_TABLE) is missing: generate it (Crystaline OS: tools/system/dt-edits.sh, run by tools/kmake.sh) or turn off CONFIG_OF_EDITS"
	@false
endif
