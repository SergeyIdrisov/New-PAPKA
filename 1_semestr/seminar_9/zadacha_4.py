class Node:
    def __init__(self, value):
        self.value = value
        self.next = None
        self.prev = None


class DoublyLinkedList:
    def __init__(self):
        self.head = None
        self.tail = None
        self.length = 0

    def __len__(self):
        return self.length

    def insert(self, value, index=None):
        new_node = Node(value)
        if self.head is None:
            self.head = new_node
            self.tail = new_node
        elif index is None or index >= self.length:
            self.tail.next = new_node
            new_node.prev = self.tail
            self.tail = new_node
        elif index == 0:
            new_node.next = self.head
            self.head.prev = new_node
            self.head = new_node
        else:
            current_node = self.head
            for _ in range(index - 1):
                current_node = current_node.next
            new_node.next = current_node.next
            new_node.prev = current_node
            current_node.next.prev = new_node
            current_node.next = new_node
        self.length += 1

    def get_value(self, index):
        if index >= self.length:
            raise IndexError("Index out of range")
        current_node = self.head
        for _ in range(index):
            current_node = current_node.next
        return current_node.value

    def delete(self, index):
        if index >= self.length:
            raise IndexError("Index out of range")
        if index == 0:
            if self.length == 1:
                self.head = None
                self.tail = None
            else:
                self.head = self.head.next
                self.head.prev = None
        elif index == self.length - 1:
            self.tail = self.tail.prev
            self.tail.next = None
        else:
            current_node = self.head
            for _ in range(index):
                current_node = current_node.next
            current_node.prev.next = current_node.next
            current_node.next.prev = current_node.prev
        self.length -= 1