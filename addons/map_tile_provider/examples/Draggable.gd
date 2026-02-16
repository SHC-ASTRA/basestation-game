extends Control
class_name Draggable

@export var global: bool = true

var dragging: bool = false
var original_down_position: Vector2


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

	if dragging:
		if global:
			position = get_global_mouse_position() - original_down_position
			dragged()
		else:
			var parent_canvas := get_parent() as CanvasItem
			if parent_canvas:
				position = parent_canvas.get_local_mouse_position() - original_down_position
				dragged()


func dragged() -> void:
	pass
