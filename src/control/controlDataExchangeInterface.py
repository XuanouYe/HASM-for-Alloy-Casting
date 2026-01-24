from typing import Dict, Any, Optional, List
from datetime import datetime
from enum import Enum
import uuid

class DataExchangeFormat:
    """
    Unified data exchange format definition.

    Usage scenarios:
    - 3D printing module -> casting module
    - Casting module -> 5-axis machining module
    - Modules -> monitoring system
    """

    class MessageType(Enum):
        """Message types."""
        EXECUTION_SEQUENCE = "executionSequence"
        PARAMETER_UPDATE = "parameterUpdate"
        STATUS_REPORT = "statusReport"
        ERROR_REPORT = "errorReport"
        QUALITY_REPORT = "qualityReport"
        DATA_REQUEST = "dataRequest"

    class DataType(Enum):
        """Data types."""
        CONFIG = "config"
        GEOMETRY = "geometry"
        PARAMETERS = "parameters"
        EXECUTION_PLAN = "executionPlan"
        MONITORING = "monitoring"
        REPORT = "report"

    @staticmethod
    def createMessage(sourceModule: str, targetModule: str, messageType: 'DataExchangeFormat.MessageType',
                      dataType: 'DataExchangeFormat.DataType', payload: Dict[str, Any], priority: int = 5,
                      metadata: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        """
        Create a data exchange message.

        Args:
            sourceModule: Source module name.
            targetModule: Target module name.
            messageType: Message type.
            dataType: Data type.
            payload: Actual data.
            priority: Priority (1-10, 10 highest).
            metadata: Additional metadata.

        Returns:
            Standard format message dictionary.
        """
        message = {
            "version": "1.0",
            "messageId": str(uuid.uuid4()),
            "timestamp": datetime.now().isoformat(),
            "sourceModule": sourceModule,
            "targetModule": targetModule,
            "messageType": messageType.value,
            "dataType": dataType.value,
            "priority": priority,
            "payload": payload,
            "metadata": metadata or {}
        }

        return message

    @staticmethod
    def validateMessage(message: Dict[str, Any]) -> tuple[bool, str]:
        """
        Validate message format.

        Returns:
            Tuple of (is valid, error message).
        """
        requiredFields = [
            'version', 'messageId', 'timestamp',
            'sourceModule', 'targetModule',
            'messageType', 'dataType', 'payload'
        ]

        for field in requiredFields:
            if field not in message:
                return False, f"Missing required field: {field}"

        # Validate message type
        validMessageTypes = [mt.value for mt in DataExchangeFormat.MessageType]
        if message['messageType'] not in validMessageTypes:
            return False, f"Invalid message type: {message['messageType']}"

        # Validate data type
        validDataTypes = [dt.value for dt in DataExchangeFormat.DataType]
        if message['dataType'] not in validDataTypes:
            return False, f"Invalid data type: {message['dataType']}"

        return True, ""

    @staticmethod
    def convertConfigToMessage(config: Dict[str, Any], sourceModule: str, targetModule: str) -> Dict[str, Any]:
        """
        Convert configuration to data exchange message.
        """
        return DataExchangeFormat.createMessage(
            sourceModule    =   sourceModule,
            targetModule    =   targetModule,
            messageType     =   DataExchangeFormat.MessageType.PARAMETER_UPDATE,
            dataType        =   DataExchangeFormat.DataType.CONFIG,
            payload         =   config,
            priority        =   8
        )


class DataExchangeQueue:
    """
    Data exchange queue - for inter-module communication.
    """

    def __init__(self):
        self.queue: List[Dict[str, Any]] = []

    def enqueue(self, message: Dict[str, Any]) -> bool:
        """
        Enqueue a message.
        """
        isValid, error = DataExchangeFormat.validateMessage(message)
        if not isValid:
            print(f"ERROR: Message validation failed - {error}")
            return False

        self.queue.append(message)
        return True

    def dequeue(self) -> Optional[Dict[str, Any]]:
        """
        Dequeue a message (FIFO).
        """
        if not self.queue:
            return None
        return self.queue.pop(0)

    def peek(self) -> Optional[Dict[str, Any]]:
        """
        Peek at the front message without removing.
        """
        if not self.queue:
            return None
        return self.queue[0]

    def getByPriority(self) -> Optional[Dict[str, Any]]:
        """
        Get the highest priority message.
        """
        if not self.queue:
            return None

        maxPriority = max(msg.get('priority', 5) for msg in self.queue)
        for i, msg in enumerate(self.queue):
            if msg.get('priority', 5) == maxPriority:
                return self.queue.pop(i)

        return None

    def size(self) -> int:
        return len(self.queue)

    def clear(self):
        self.queue.clear()

class MessageRouter:
    """
    Message routing between modules - ensures correct message delivery.
    """

    def __init__(self):
        self.queues: Dict[str, DataExchangeQueue] = {}
        self.messageLog = []

    def registerModule(self, moduleName: str):
        """
        Register a module with its own message queue.
        """
        if moduleName not in self.queues:
            self.queues[moduleName] = DataExchangeQueue()

    def sendMessage(
        self,
        message: Dict[str, Any],
        targetModule: Optional[str] = None
    ) -> bool:
        """Send message to target module."""
        if not targetModule:
            targetModule = message.get('targetModule')

        if targetModule not in self.queues:
            print(f"ERROR: Target module '{targetModule}' not registered")
            return False

        success = self.queues[targetModule].enqueue(message)
        self.messageLog.append({
            "timestamp": datetime.now().isoformat(),
            "message": message,
            "status": "sent" if success else "failed"
        })

        return success

    def receiveMessage(self, moduleName: str) -> Optional[Dict[str, Any]]:
        """
        Receive message by priority for a module.
        """
        if moduleName not in self.queues:
            return None

        return self.queues[moduleName].getByPriority()

    def getMessageLog(self) -> List[Dict]:
        """
        Get all message logs.
        """
        return self.messageLog


def main():
    """Test data exchange interface."""
    print("=== Data Exchange Interface Test ===")

    # 1. Test message creation
    print("1. Testing message creation...")
    payload = {
        "parameter": "layerHeight",
        "value": 0.2,
        "unit": "mm"
    }

    message = DataExchangeFormat.createMessage(
        sourceModule="additive_module",
        targetModule="casting_module",
        messageType=DataExchangeFormat.MessageType.PARAMETER_UPDATE,
        dataType=DataExchangeFormat.DataType.PARAMETERS,
        payload=payload,
        priority=8
    )

    print(f"   Message ID: {message.get('messageId')}")
    print(f"   Source module: {message.get('sourceModule')}")
    print(f"   Target module: {message.get('targetModule')}")
    print(f"   Message type: {message.get('messageType')}")
    print(f"   Data type: {message.get('dataType')}")

    # 2. Test message validation
    print("\n2. Testing message validation...")
    isValid, error = DataExchangeFormat.validateMessage(message)
    print(f"   Message validation: {'Passed' if isValid else f'Failed: {error}'}")

    # Test invalid message
    invalidMessage = {"timestamp": "2024-01-01"}
    isValid, error = DataExchangeFormat.validateMessage(invalidMessage)
    print(f"   Invalid message validation: {'Passed' if isValid else f'Failed: {error}'}")

    # 3. Test queue functionality
    print("\n3. Testing queue functionality...")
    queue = DataExchangeQueue()

    # Add message to queue
    success = queue.enqueue(message)
    print(f"   Add message to queue: {'Success' if success else 'Failed'}")

    # Create another message
    message2 = DataExchangeFormat.createMessage(
        sourceModule="casting_module",
        targetModule="subtractive_module",
        messageType=DataExchangeFormat.MessageType.STATUS_REPORT,
        dataType=DataExchangeFormat.DataType.MONITORING,
        payload={"status": "completed", "temperature": 200},
        priority=10  # Higher priority
    )
    queue.enqueue(message2)

    print(f"   Queue size: {queue.size()}")

    # 4. Test priority-based retrieval
    print("\n4. Testing priority-based retrieval...")
    highPriorityMsg = queue.getByPriority()
    print(f"   Get highest priority message: {highPriorityMsg.get('priority') if highPriorityMsg else 'No messages'}")

    # 5. Test message routing
    print("\n5. Testing message routing...")
    router = MessageRouter()

    # Register modules
    router.registerModule("additive_module")
    router.registerModule("casting_module")
    router.registerModule("subtractive_module")

    print(f"   Registered modules: {list(router.queues.keys())}")

    # Send message
    success = router.sendMessage(message)
    print(f"   Send message: {'Success' if success else 'Failed'}")

    # Receive message
    receivedMsg = router.receiveMessage("casting_module")
    print(f"   Receive message: {'Success' if receivedMsg else 'No messages'}")

    # 6. Test configuration conversion
    print("\n6. Testing configuration conversion...")
    config = {
        "layerHeight": 0.2,
        "nozzleTemperature": 210,
        "printSpeed": 100
    }

    configMessage = DataExchangeFormat.convertConfigToMessage(
        config=config,
        sourceModule="config_manager",
        targetModule="additive_module"
    )

    print(f"   Config message type: {configMessage.get('messageType')}")
    print(f"   Config message priority: {configMessage.get('priority')}")

    # 7. View logs
    print("\n7. Viewing message logs...")
    logs = router.getMessageLog()
    print(f"   Message log count: {len(logs)}")
    for log in logs:
        print(f"   Time: {log.get('timestamp')}, Status: {log.get('status')}")

    print("\nTest completed!")


if __name__ == "__main__":
    main()