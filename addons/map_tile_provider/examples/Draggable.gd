extends Control
class_name Draggable

@export var global: bool = true

var dragging: bool = false
var original_down_position: Vector2
var parent_canvas: CanvasItem
var drag_offset: Vector2

func _ready() -> void:
	parent_canvas = get_parent() as CanvasItem
	drag_offset = size
	drag_offset.x *= 0.5

func _gui_input(event: InputEvent) -> void:
	if event is InputEventMouseButton:
		var mb := event as InputEventMouseButton
		if mb.is_released():
			dragging = false
		elif mb.is_pressed():
			dragging = true
			original_down_position = get_local_mouse_position()
		else:
			_input(event)
	if event is InputEventMouseMotion and dragging:
		var mm := event as InputEventMouseMotion
		if global:
			position = mm.global_position - original_down_position
			dragged()
		else:
			position = parent_canvas.get_local_mouse_position() - drag_offset
			dragged()


func dragged() -> void:
	pass
