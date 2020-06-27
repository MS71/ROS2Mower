PROJECT_NAME := esp32-cam-demo

include $(IDF_PATH)/make/project.mk

doota: app
	curl 192.168.1.75:8032 --data-binary @- < build/$(PROJECT_NAME).bin

udplog:
	nc -u -l 30000
