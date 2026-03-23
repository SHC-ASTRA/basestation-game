extends Draggable
class_name MapPin

@export var pin_button : Button
@export var popup_panel: Panel
@export var panel_name: LineEdit
@export var panel_name_presets: OptionButton
@export var x: Label
@export var y: Label
@export var delete_button: Button
@export var under_pin: Label

var svc: MapSVC
var pinCoordinates : Vector2

func _ready() -> void:
	super._ready()
	svc = (get_parent().get_parent().get_child(0) as MapSVC)

	pin_button.toggled.connect(func(t: bool)-> void:
		popup_panel.visible = t
	)
	panel_name_presets.item_selected.connect(func(_index: int)-> void:
		panel_name.text = panel_name_presets.text
		under_pin.text = panel_name_presets.text
	)
	panel_name.text_changed.connect(func(new_text: String)-> void:
		under_pin.text = new_text
	)
	delete_button.pressed.connect(func()-> void:
		queue_free()
	)


func dragged() -> void:
	svc.loadMousePos()
	svc.updateMap()
	pinCoordinates = svc.posToCoords(svc.normMousePos * svc.sv_rect.size)
	x.text = "X: %f" % pinCoordinates.y
	y.text = "Y: %f" % pinCoordinates.x
	position = svc.normMousePos * svc.sv_rect.size
