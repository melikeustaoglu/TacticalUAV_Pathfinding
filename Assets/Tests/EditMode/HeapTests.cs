using System;
using System.Collections.Generic;
using NUnit.Framework;
using UnityEngine;

[TestFixture]
public class HeapTests
{
    private class TestItem : IHeapItem<TestItem>
    {
        public int Cost;
        public int HeapIndex { get; set; }

        public TestItem(int cost)
        {
            Cost = cost;
        }

        public int CompareTo(TestItem other)
        {
            // Reverse compare to match Node.cs min-heap convention
            int compare = Cost.CompareTo(other.Cost);
            return -compare;
        }
    }

    [Test]
    public void Heap_AddAndExtract_ReturnsItemsInAscendingOrder()
    {
        Heap<TestItem> heap = new Heap<TestItem>(10);
        int[] input = new int[] { 45, 12, 89, 3, 27, 60, 1, 99, 15 };

        for (int i = 0; i < input.Length; i++)
        {
            heap.Add(new TestItem(input[i]));
        }

        Assert.AreEqual(input.Length, heap.Count);

        List<int> extracted = new List<int>();
        while (heap.Count > 0)
        {
            extracted.Add(heap.RemoveFirst().Cost);
        }

        int[] expected = new int[] { 1, 3, 12, 15, 27, 45, 60, 89, 99 };
        Assert.AreEqual(expected, extracted.ToArray());
    }

    [Test]
    public void Heap_DecreaseKey_CorrectlyPromotesUpdatedItem()
    {
        Heap<TestItem> heap = new Heap<TestItem>(10);
        TestItem itemA = new TestItem(50);
        TestItem itemB = new TestItem(30);
        TestItem itemC = new TestItem(70);

        heap.Add(itemA);
        heap.Add(itemB);
        heap.Add(itemC);

        // Decrease key of itemA from 50 down to 10 (should now become top item)
        itemA.Cost = 10;
        heap.UpdateItem(itemA);

        TestItem first = heap.RemoveFirst();
        Assert.AreEqual(itemA, first);
        Assert.AreEqual(10, first.Cost);
    }

    [Test]
    public void Heap_Contains_AccuratelyTracksHeapMembership()
    {
        Heap<TestItem> heap = new Heap<TestItem>(5);
        TestItem item1 = new TestItem(10);
        TestItem item2 = new TestItem(20);

        Assert.IsFalse(heap.Contains(item1));
        heap.Add(item1);
        Assert.IsTrue(heap.Contains(item1));
        Assert.IsFalse(heap.Contains(item2));

        heap.RemoveFirst();
        Assert.IsFalse(heap.Contains(item1));
    }

    [Test]
    public void Heap_NodeCompareTo_PrioritizesLowestFCostAndBreaksTiesWithHCost()
    {
        Heap<Node> heap = new Heap<Node>(10);
        Node nodeA = new Node(true, Vector3.zero, 0, 0) { gCost = 10, hCost = 20 }; // fCost = 30, hCost = 20
        Node nodeB = new Node(true, Vector3.zero, 1, 0) { gCost = 5, hCost = 15 };  // fCost = 20, hCost = 15
        Node nodeC = new Node(true, Vector3.zero, 2, 0) { gCost = 20, hCost = 10 }; // fCost = 30, hCost = 10 (Tie-breaker winner vs A)

        heap.Add(nodeA);
        heap.Add(nodeB);
        heap.Add(nodeC);

        Assert.AreEqual(nodeB, heap.RemoveFirst()); // fCost 20
        Assert.AreEqual(nodeC, heap.RemoveFirst()); // fCost 30, hCost 10
        Assert.AreEqual(nodeA, heap.RemoveFirst()); // fCost 30, hCost 20
    }
}
