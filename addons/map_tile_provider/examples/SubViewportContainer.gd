extends SubViewportContainer


@onready var _map = $SubViewport/Map

@export var popupNode: PackedScene

var lastValidMousePos: Vector2

var pinsParent: Control

var focused: bool

# Called when the node enters the scene tree for the first time.
func _ready():
	_map.recenter()
	pinsParent = get_parent().get_child(-1);

func _input(event):
	# Get the SubViewport global rect (screen coords)
	var sv_rect = get_global_rect()

	# Calculate mouse position relative to the SubViewport
	var mouse_local_pos = get_viewport().get_mouse_position() - sv_rect.position

	# Normalize position inside the SubViewport rect (clamped 0..1)
	var norm_pos = Vector2(
		clamp(mouse_local_pos.x / sv_rect.size.x, 0, 1),
		clamp(mouse_local_pos.y / sv_rect.size.y, 0, 1)
	)
	if focused and event is InputEventMouseMotion:
		if event.button_mask == 1:
			_map.shift(event.relative)
		else:
			if norm_pos.x <= 0 or norm_pos.x >= 1 or norm_pos.y <= 0 or norm_pos.y >= 1:
				return

			lastValidMousePos = norm_pos

			var layer = _map._zooms[int(_map.zoom)]["layer"]
			if layer == null:
				return # skip layer 0

			# Get center tile from Map's latitude/longitude
			var center_tile = _map.loader.gps_to_tile(_map.latitude, _map.longitude, int(_map.zoom))
			var key = "%d,%d" % [center_tile.x, center_tile.y]

			var deg_per_pix: Vector2
			var degrees: Vector2

			if key in layer:
				var tile = layer[key]
				degrees = tile["bounds"].size
				var step = tile["sprite"].texture.get_size()
				deg_per_pix = Vector2(degrees.x / step.x, degrees.y / step.y)
			else:
				degrees = _map.loader.get_tile_bounds(center_tile.x, center_tile.y, center_tile.z).size
				deg_per_pix = Vector2(degrees.x / 256.0, degrees.y / 256.0)


			# Offset from the center of the Map
			var offset = norm_pos * (_map._size as Vector2) - ((_map._size as Vector2) * 0.5)

			# Update the labels on Map node
			_map.latitudeLabel.text = "Latitude: %f" % (_map.latitude + offset.y * deg_per_pix.y)
			_map.longitudeLabel.text = "Longitude: %f" % (_map.longitude + offset.x * deg_per_pix.x)

	elif event is InputEventMouseButton:
		if focused and event.button_mask == 1:
			if norm_pos.x <= 0 or norm_pos.x >= 1 or norm_pos.y <= 0 or norm_pos.y >= 1:
				return
			var ButtonPopup = popupNode.instantiate()
			pinsParent.add_child(ButtonPopup)
		elif event.button_mask == 8:
			_map.zoom += 1
		elif event.button_mask == 16:
			_map.zoom -= 1


func _on_focus_entered() -> void:
	focused = true


func _on_focus_exited() -> void:
	focused = false
