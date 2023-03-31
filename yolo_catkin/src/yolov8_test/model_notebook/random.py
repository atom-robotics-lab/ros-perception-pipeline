# Given an array of N integers and a number K, the task is to find the number of pairs of integers in the array whose sum is equal to K.

def search(arr,k):
    freq={}
    count=0
    for i in arr:
        if (i in freq):
            freq[i]+=1 
        else:
            freq[i]=1
    for i in arr:
        count+=freq[k-i]
        if i==k-i:   
            count-=1
    count/=2
    return count
if __name__=='__main__':
    print(search([1, 5, 7, -1, 5,2,4],6))
    