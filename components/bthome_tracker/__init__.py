import esphome.codegen as cg

bthome_tracker_ns = cg.esphome_ns.namespace("bthome_tracker")
BTHomeTracker = bthome_tracker_ns.class_("BTHomeTracker", cg.Component)

CONFIG_SCHEMA = cg.Component.CONFIG_SCHEMA

def to_code(config):
    var = cg.new_Pvariable(config[cg.GenerateID()])
    yield cg.register_component(var, config)