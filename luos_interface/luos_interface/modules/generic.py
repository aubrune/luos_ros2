class LuosGenericPublisher(object):
    QUEUE_SIZE = 10
    RATE_HZ = 20
    def __init__(self, node, module, variables, events):
        self._node = node
        self._module = module
        self._publishers = {}
        self._timer = None
        self.variables = variables   # Dict {name: ROSType}
        self.events = events         # Dict {name: ROSType}

        # Open publishers for Luos variables
        for variable in self.variables:
            type = self.variables[variable]["type"]
            topic = "/".join([module.alias, "variables", variable, "read"])
            self._publishers[variable] = {
                "topic": topic,
                "type": type,
                "pub": self._node.create_publisher(type, topic, self.QUEUE_SIZE)
            }
        
        self._timer = self._node.create_timer(1./self.RATE_HZ, self._timer_callback)

        # Open publishers for Luos events
        for event in self.events:
            type = self.events[event]["type"]
            topic = "/".join([module.alias, "events", event])
            serialize = self.events[event]["serialize"]
            self._publishers[event] = self._node.create_publisher(type, topic, self.QUEUE_SIZE)
            module.add_callback(event, lambda e: self._publishers[e.name].publish(serialize(module, e)))

    def _timer_callback(self):
        for variable, info in self.variables.items():
            serialize = info["serialize"]
            self._publishers[variable]["pub"].publish(serialize(self._module, getattr(self._module, variable)))