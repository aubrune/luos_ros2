class LuosGenericPublisher(object):
    QUEUE_SIZE = 1
    def __init__(self, node, module, rate, variables, events, aggregates):
        self._node = node
        self._module = module
        self._publishers = {}
        self._subscribers = {}
        self._timer = None
        self._rate = rate
        self.variables = variables    # Dict {name: ROSType}
        self.events = events          # Dict {name: ROSType}
        self.aggregates = aggregates  # Dict {name: ROSType}

        # Open publishers for Luos variables
        for variable, info in self.variables.items():
            type = info["type"]
            topic_root = [module.alias, "variables", variable]
            if "read" in info and info["read"]:
                topic = "/".join(topic_root + ["read"])
                self._publishers[variable] = {
                    "topic": topic,
                    "type": type,
                    "pub": self._node.create_publisher(type, topic, self.QUEUE_SIZE)
                }
            if "write" in info and info["write"]:
                topic = "/".join(topic_root + ["write"])
                callback = lambda msg, variable=variable: self._subscription_callback(msg, variable)
                self._subscribers[variable] = {
                    "topic": topic,
                    "type": type,
                    "callback": callback,
                }
                self._node.create_subscription(
                    type,
                    topic,
                    callback,
                    self.QUEUE_SIZE
                )

        # Open publishers for aggregated Luos variables converted into ROS standard messages
        for aggregate, info in self.aggregates.items():
            type = info["type"]
            topic = "/".join([module.alias, aggregate])
            serialize = info["serialize"]
            self._publishers[aggregate] = {
                "topic": topic,
                "type": type,
                "pub": self._node.create_publisher(type, topic, self.QUEUE_SIZE)
            }

        # Open publishers for Luos events
        for event, info in self.events.items():
            type = info["type"]
            topic = "/".join([module.alias, "events", event])
            serialize = info["serialize"]
            self._publishers[event] = {
                "topic": topic,
                "type": type,
                "pub": self._node.create_publisher(type, topic, self.QUEUE_SIZE)
            }
            module.add_callback(event, lambda e: self._publishers[event]["pub"].publish(serialize(module, e)))

        # Start the timer for variables and aggregates
        self._timer = self._node.create_timer(1./self._rate, self._timer_callback)

    def _subscription_callback(self, msg, variable):
        # A "write" message is incoming, it is transferred to Luos modules
        deserialize = self.variables[variable]["deserialize"]
        setattr(self._module, variable, deserialize(msg))

    def _timer_callback(self):
        # "read" messages are all sent at a specific rate
        # Publish variables on a regular basis (with module-agnostic serializers e.g. Float32, UInt32...)
        for variable, info in self.variables.items():
            serialize = info["serialize"]
            self._publishers[variable]["pub"].publish(serialize(getattr(self._module, variable)))
        
        # Publish aggregates on a regular basis (with module-dependent serializers)
        for aggregate, info in self.aggregates.items():
            serialize = info["serialize"]
            self._publishers[aggregate]["pub"].publish(serialize(self._module))