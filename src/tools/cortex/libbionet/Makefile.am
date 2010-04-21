
noinst_LIBRARIES = libbionet.a
libbionet_a_SOURCES = ./cache-internal.c  \
cache-lookup-hab.c  \
cache-lookup-node.c  \
cache-lookup-resource.c  \
connect-to-nag.c  \
get-nag-error.c  \
global.c  \
handle-error-message.c  \
handle-queued-nag-messages.c  \
kill-nag-connection.c  \
list-habs.c  \
list-nodes.c  \
parse-hab-from-xml.c  \
parse-node-from-xml.c  \
parse-resource.c  \
queue-nag-message.c  \
read-from-nag.c  \
read-ok-from-nag.c  \
register-callback.c  \
set-id.c  \
set-nag-hostname.c  \
set-resource.c  \
subscribe-hab-list.c  \
subscribe-node-list.c  \
subscribe-resource.c  \
../util/argument.c \
../util/command.c \
../util/glib-log-handler.c \
../util/hab-add-node.c \
../util/hab-free.c \
../util/hab-get-node.c \
../util/hab-matches-type-and-id.c \
../util/hab-new.c \
../util/hab-remove-node.c \
../util/is-valid-name-component.c \
../util/log-hexdump.c \
../util/node-add-command.c \
../util/node-add-resource.c \
../util/node-add-stream.c \
../util/node-free.c \
../util/node-get-resource.c \
../util/node-get-stream.c \
../util/node-matches.c \
../util/node-new.c \
../util/node-set-resource-value.c \
../util/nxio-misc.c \
../util/nxio-read.c \
../util/nxio-write.c \
../util/resource-data-type.c \
../util/resource-flavor.c \
../util/resource-free.c \
../util/resource-matches.c \
../util/resource-new.c \
../util/resource-time.c \
../util/resource-value.c \
../util/stream-accept.c \
../util/stream-connect.c \
../util/stream-direction.c \
../util/stream-free.c \
../util/stream-listen.c \
../util/stream-new.c \
../util/stream-port.c

libbionet_a_CFLAGS = -I. -I$(top_srcdir)/src/tools/cortex/util `xml2-config --cflags` `pkg-config --cflags glib-2.0`
#libbionet_a_LDFLAGS = -L. `xml1-config --libs` `pkg-config --libs glib-2.0`
