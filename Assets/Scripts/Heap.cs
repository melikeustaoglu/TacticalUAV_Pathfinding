using System;

/// <summary>
/// Interface required by items stored in the generic Heap data structure.
/// </summary>
/// <typeparam name="T">Type of the heap item.</typeparam>
public interface IHeapItem<T> : IComparable<T>
{
    int HeapIndex { get; set; }
}

/// <summary>
/// High-performance generic Binary Min-Heap priority queue.
/// Provides O(log N) extraction, insertion, and update (decrease-key),
/// and O(1) heap membership lookup via cached heap indexing.
/// </summary>
/// <typeparam name="T">Heap item type implementing IHeapItem.</typeparam>
public class Heap<T> where T : IHeapItem<T>
{
    private T[] items;
    private int currentItemCount;

    public int Count => currentItemCount;

    public Heap(int maxHeapSize)
    {
        items = new T[Math.Max(1, maxHeapSize)];
        currentItemCount = 0;
    }

    public void Add(T item)
    {
        item.HeapIndex = currentItemCount;
        items[currentItemCount] = item;
        SortUp(item);
        currentItemCount++;
    }

    public T RemoveFirst()
    {
        if (currentItemCount == 0)
            return default;

        T firstItem = items[0];
        currentItemCount--;
        items[0] = items[currentItemCount];
        items[0].HeapIndex = 0;
        SortDown(items[0]);
        return firstItem;
    }

    /// <summary>
    /// Re-evaluates item priority when its cost decreases (decrease-key).
    /// </summary>
    public void UpdateItem(T item)
    {
        SortUp(item);
    }

    /// <summary>
    /// Fast O(1) membership check using item's cached heap index.
    /// </summary>
    public bool Contains(T item)
    {
        if (item == null || item.HeapIndex < 0 || item.HeapIndex >= currentItemCount)
            return false;

        return Equals(items[item.HeapIndex], item);
    }

    public void Clear()
    {
        for (int i = 0; i < currentItemCount; i++)
        {
            if (items[i] != null)
            {
                items[i].HeapIndex = -1;
            }
            items[i] = default;
        }
        currentItemCount = 0;
    }

    private void SortUp(T item)
    {
        int parentIndex = (item.HeapIndex - 1) / 2;

        while (item.HeapIndex > 0)
        {
            T parentItem = items[parentIndex];
            if (item.CompareTo(parentItem) > 0)
            {
                Swap(item, parentItem);
            }
            else
            {
                break;
            }

            parentIndex = (item.HeapIndex - 1) / 2;
        }
    }

    private void SortDown(T item)
    {
        while (true)
        {
            int childIndexLeft = item.HeapIndex * 2 + 1;
            int childIndexRight = item.HeapIndex * 2 + 2;
            int swapIndex = 0;

            if (childIndexLeft < currentItemCount)
            {
                swapIndex = childIndexLeft;

                if (childIndexRight < currentItemCount)
                {
                    if (items[childIndexRight].CompareTo(items[childIndexLeft]) > 0)
                    {
                        swapIndex = childIndexRight;
                    }
                }

                if (items[swapIndex].CompareTo(item) > 0)
                {
                    Swap(item, items[swapIndex]);
                }
                else
                {
                    return;
                }
            }
            else
            {
                return;
            }
        }
    }

    private void Swap(T itemA, T itemB)
    {
        items[itemA.HeapIndex] = itemB;
        items[itemB.HeapIndex] = itemA;
        int itemAIndex = itemA.HeapIndex;
        itemA.HeapIndex = itemB.HeapIndex;
        itemB.HeapIndex = itemAIndex;
    }
}
