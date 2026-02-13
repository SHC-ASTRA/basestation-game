extends Draggable
class_name MapPin

@export var popup_panel: Panel
@export var panel_name: LineEdit
@export var panel_name_presets: OptionButton
@export var x: Label
@export var y: Label
@export var delete_button: Button
@export var under_pin: Label

var svc: SubViewportContainer

func _ready() -> void:
	svc = (get_parent().get_parent().get_child(-2) as SubViewportContainer)
	var btn := self as Control as Button

	if btn:
		btn.toggled.connect(func(t: bool)-> void:
			popup_panel.visible = t
		)
	panel_name_presets.item_selected.connect(func(_index: int)-> void:
		panel_name.text = panel_name_presets.text
		under_pin.text = panel_name_presets.text
	)
	delete_button.pressed.connect(func()-> void:
		queue_free()
	)
	panel_name.text_changed.connect(func(new_text: String)-> void:
		under_pin.text = new_text
	)

	var global_mouse: Vector2 = svc.get_global_mouse_position()
	global_position = global_mouse

	position -= Vector2(16, 42)


func dragged() -> void:
	# Get the Control global rect (screen coords)
	var sv_rect: Rect2 = svc.get_global_rect()

	# Mouse position relative to the Control
	var mouse_local_pos: Vector2 = svc.get_viewport().get_mouse_position() - sv_rect.position

	# Normalize position inside the rect (clamped 0..1)
	var norm_pos := Vector2(
		clamp(mouse_local_pos.x / sv_rect.size.x, 0.0, 1.0),
		clamp(mouse_local_pos.y / sv_rect.size.y, 0.0, 1.0)
	)

	var p = svc.getPosOnMap(norm_pos)

	x.text = "X: %f" % p.y
	y.text = "Y: %f" % p.x
