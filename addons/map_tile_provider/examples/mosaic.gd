extends Control


signal zoom_changed(float)
signal latitude_changed(float)
signal longitude_changed(float)

@export var latitude: float = 38.37608:
	set(val):
		latitude = val
		latitude_changed.emit(val)
@export var longitude: float = -110.71710:
	set(val):
		longitude = val
		longitude_changed.emit(val)
@export_range(1, 20) var zoom: float = 14:
	set(val):
		zoom = val
		zoom_changed.emit(val)

@onready var _map = $VBoxContainer/SubViewportContainer/SubViewport/Map

# Called when the node enters the scene tree for the first time.
func _ready():
	_map.zoom = zoom
	_map.latitude = latitude
	_map.longitude = longitude


func _on_map_latitude_changed(val: float):
	latitude = val


func _on_map_longitude_changed(val: float):
	longitude = val


func _on_map_zoom_changed(val: float):
	zoom = val
