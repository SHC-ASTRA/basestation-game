extends SubViewportContainer
class_name MapSVC

@onready var _map = $SubViewport/Map
@onready var _cam = $SubViewport/Camera2D
@export var popupNode: PackedScene
var pinsParent: Control
var sv_rect : Rect2
var normMousePos : Vector2
# var posfromcoords
# var coordsfrompos
# var mousepos

# Called when the node enters the scene tree for the first time.
func _ready():
	# posfromcoords = FileAccess.open("/tmp/posfromcoords", FileAccess.READ_WRITE);
	# coordsfrompos = FileAccess.open("/tmp/coordsfrompos", FileAccess.READ_WRITE);
	# mousepos = FileAccess.open("/tmp/mousepos", FileAccess.READ_WRITE);
	# posfromcoords.seek_end()
	# coordsfrompos.seek_end()
	# mousepos.seek_end()

	_map.recenter()
	loadMousePos()
	pinsParent = get_parent().get_child(1);

func _gui_input(event):
	loadMousePos()

	if event is InputEventMouseMotion:
		updateMap()
		if event.button_mask == 1:
			_map.shift(event.relative)
			updateMap()

			for pin in pinsParent.get_children() as Array[MapPin]:
				pin.position = coordsToPos(pin.pinCoordinates)
				# setPinVisibility(pin, sv_rect.size)

		else:
			if normMousePos.x <= 0 or normMousePos.x >= 1 or normMousePos.y <= 0 or normMousePos.y >= 1:
				_map.latitudeLabel.text = "Latitude: %f" % _map.latitude
				_map.longitudeLabel.text = "Longitude: %f" % _map.longitude
				return

			# Update the labels on Map node
			var posOnMap : Vector2 = posToCoords(normMousePos)
			_map.latitudeLabel.text = "Latitude: %f" % posOnMap.y
			_map.longitudeLabel.text = "Longitude: %f" % posOnMap.x

	elif event is InputEventMouseButton:
		if event.pressed and event.button_mask == 1:
			if normMousePos.x <= 0 or normMousePos.x >= 1 or normMousePos.y <= 0 or normMousePos.y >= 1:
				return

			if event.double_click:
				var pin = popupNode.instantiate() as MapPin
				pinsParent.add_child(pin)
				var posOnMap : Vector2 = posToCoords(normMousePos)
				pin.position = coordsToPos(posOnMap)
				pin.pinCoordinates = posOnMap
				pin.x.text = "X: %f" % posOnMap.y
				pin.y.text = "Y: %f" % posOnMap.x

		elif event.button_mask == 8:
			if _map.zoom == 20:
				_cam.zoom += Vector2.ONE * get_process_delta_time()
				return
			else:
				_map.zoom += 1

			updateMap()
			var posOnMap : Vector2 = posToCoords(normMousePos)

			for pin in pinsParent.get_children() as Array[MapPin]:
				pin.position = coordsToPos(pin.pinCoordinates)
				setPinVisibility(pin, sv_rect.size)

		elif event.button_mask == 16:
			if _map.zoom == 20 and not _cam.zoom == Vector2.ONE:
				_cam.zoom = Vector2.ONE
				return
			else:
				_map.zoom -= 1

			updateMap()
			var posOnMap : Vector2 = posToCoords(normMousePos)

			for pin in pinsParent.get_children() as Array[MapPin]:
				pin.position = coordsToPos(pin.pinCoordinates)
				setPinVisibility(pin, sv_rect.size)

var deg_per_pix: Vector2

func updateMap() -> void:
	var layer = _map._zooms[int(_map.zoom)]["layer"]
	if layer == null:
		return Vector2() # skip layer 0

	# Get center tile from Map's latitude/longitude
	var center_tile = _map.loader.gps_to_tile(_map.latitude, _map.longitude, int(_map.zoom))
	var key = "%d,%d" % [center_tile.x, center_tile.y]

	var degrees: Vector2
	if key in layer:
		var tile = layer[key]
		degrees = tile["bounds"].size
		var step = tile["sprite"].texture.get_size()
		deg_per_pix = Vector2(degrees.x / step.x, degrees.y / step.y)
	else:
		degrees = _map.loader.get_tile_bounds(center_tile.x, center_tile.y, center_tile.z).size
		deg_per_pix = Vector2(degrees.x * 0.00390625, degrees.y * 0.00390625)
		# 0.00390625 is 1/256, mul is faster than div

func coordsToPos(in_pos: Vector2) -> Vector2:
	var pos = Vector2(
		(in_pos.x - _map.longitude) / deg_per_pix.x,
		(_map.latitude - in_pos.y) / deg_per_pix.y
	)
	# posfromcoords.store_line("x: (%f - %f) / %f = %f" % [in_pos.x, _map.longitude, deg_per_pix.x, pos.x])
	# posfromcoords.store_line("y: (%f - %f) / %f = %f" % [in_pos.y, _map.latitude, deg_per_pix.y, pos.y])
	return pos

func posToCoords(in_pos: Vector2) -> Vector2:
	var coords = Vector2(
		_map.longitude + (in_pos.x) * deg_per_pix.x,
		_map.latitude + (in_pos.y - sv_rect.size.y * 0.25) * deg_per_pix.y
	)
	# coordsfrompos.store_line("%f, %f" % [coords.x, coords.y])
	return coords

func setPinVisibility(pin : MapPin, sv_rect_size : Vector2) -> void:
	var pin_hyperlocal_pos : Vector2 = pin.position + pinsParent.position
	var pin_norm_pos = Vector2(
		clamp(pin_hyperlocal_pos.x / sv_rect_size.x, 0, 1),
		clamp(pin_hyperlocal_pos.y / sv_rect_size.y, 0, 1)
	)
	pin.visible = pin_norm_pos.x > 0 and pin_norm_pos.x < 1 and pin_norm_pos.y > 0 and pin_norm_pos.y < 1

func loadMousePos():
	sv_rect = get_global_rect()
	var mouse_local_pos = get_viewport().get_mouse_position() - sv_rect.position
	normMousePos = Vector2(
		clamp(mouse_local_pos.x / sv_rect.size.x, 0, 1),
		clamp(mouse_local_pos.y / sv_rect.size.y, 0, 1)
	)
	# mousepos.store_line("%f, %f" % [mouse_local_pos.x, mouse_local_pos.y])
