class LuosGenericPublisher(object):
    QUEUE_SIZE = 1
    RATE_HZ = 20
    def __init__(self, node, module, variables, events, aggregates):
        self._node = node
        self._module = module
        self._publishers = {}
        self._timer = None
        self.variables = variables    # Dict {name: ROSType}
        self.events = events          # Dict {name: ROSType}
        self.aggregates = aggregates  # Dict {name: ROSType}

        # Open publishers for Luos variables
        for variable in self.variables:
            type = self.variables[variable]["type"]
            topic = "/".join([module.alias, "variables", variable, "read"])
            self._publishers[variable] = {
                "topic": topic,
                "type": type,
                "pub": self._node.create_publisher(type, topic, self.QUEUE_SIZE)
            }

        # Open publishers for aggregated Luos variables converted into ROS standard messages
        for aggregate in self.aggregates:
            type = self.aggregates[aggregate]["type"]
            topic = "/".join([module.alias, aggregate])
            serialize = self.aggregates[aggregate]["serialize"]
            self._publishers[aggregate] = {
                "topic": topic,
                "type": type,
                "pub": self._node.create_publisher(type, topic, self.QUEUE_SIZE)
            }

        # Open publishers for Luos events
        for event in self.events:
            type = self.events[event]["type"]
            topic = "/".join([module.alias, "events", event])
            serialize = self.events[event]["serialize"]
            self._publishers[event] = {
                "topic": topic,
                "type": type,
                "pub": self._node.create_publisher(type, topic, self.QUEUE_SIZE)
            }
            module.add_callback(event, lambda e: self._publishers[event]["pub"].publish(serialize(module, e)))

        # Start the timer for variables and aggregates
        self._timer = self._node.create_timer(1./self.RATE_HZ, self._timer_callback)

    def _timer_callback(self):
        # Publish variables on a regular basis (with module-agnostic serializers e.g. Float32, UInt32...)
        for variable, info in self.variables.items():
            serialize = info["serialize"]
            self._publishers[variable]["pub"].publish(serialize(getattr(self._module, variable)))
        
        # Publish aggregates on a regular basis (with module-dependent serializers)
        for aggregate, info in self.aggregates.items():
            serialize = info["serialize"]
            self._publishers[aggregate]["pub"].publish(serialize(self._module))