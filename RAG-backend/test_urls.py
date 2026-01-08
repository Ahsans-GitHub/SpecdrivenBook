#!/usr/bin/env python3
"""
Script to test which URLs from your sitemap are actually accessible
"""

import requests
from urllib.parse import urljoin
import time

def test_urls():
    """Test which URLs are actually accessible."""
    base_url = "https://physicalaiandhumanoidrobotics.vercel.app"

    # Chapter URLs extracted from the sitemap
    urls = [
        f"{base_url}/",
        f"{base_url}/docs",
        f"{base_url}/docs/chapter1",
        f"{base_url}/docs/chapter1/lesson1",
        f"{base_url}/docs/chapter1/lesson2",
        f"{base_url}/docs/chapter1/lesson3",
        f"{base_url}/docs/chapter1/lesson4",
        f"{base_url}/docs/chapter2/chapter2-overview",
        f"{base_url}/docs/chapter2/module1-overview",
        f"{base_url}/docs/chapter2/module1/lesson1",
        f"{base_url}/docs/chapter2/module1/lesson2",
        f"{base_url}/docs/chapter2/module1/lesson3",
        f"{base_url}/docs/chapter2/module1/lesson4",
        f"{base_url}/docs/chapter3/chapter3-overview",
        f"{base_url}/docs/chapter3/module2-overview",
        f"{base_url}/docs/chapter3/module2/lesson1",
        f"{base_url}/docs/chapter3/module2/lesson2",
        f"{base_url}/docs/chapter3/module2/lesson3",
        f"{base_url}/docs/chapter3/module2/lesson4",
        f"{base_url}/docs/chapter4/chapter4-overview",
        f"{base_url}/docs/chapter4/module3-overview",
        f"{base_url}/docs/chapter4/module3/lesson1",
        f"{base_url}/docs/chapter4/module3/lesson2",
        f"{base_url}/docs/chapter4/module3/lesson3",
        f"{base_url}/docs/chapter4/module3/lesson4",
        f"{base_url}/docs/chapter5/chapter5-overview",
        f"{base_url}/docs/chapter5/module4-overview",
        f"{base_url}/docs/chapter5/module4/lesson1",
        f"{base_url}/docs/chapter5/module4/lesson2",
        f"{base_url}/docs/chapter5/module4/lesson3",
        f"{base_url}/docs/chapter5/module4/lesson4",
        f"{base_url}/docs/chapter6",
        f"{base_url}/docs/chapter6/lesson1",
        f"{base_url}/docs/chapter6/lesson2",
        f"{base_url}/docs/chapter6/lesson3",
        f"{base_url}/docs/chapter7",
        f"{base_url}/docs/chapter7/assessment1",
        f"{base_url}/docs/chapter7/assessment2",
        f"{base_url}/docs/chapter7/assessment3",
        f"{base_url}/docs/chapter7/assessment4",
        f"{base_url}/docs/hardware",
        f"{base_url}/docs/hardware-requirements"
    ]

    accessible_urls = []
    inaccessible_urls = []

    print("Testing URL accessibility...")
    for i, url in enumerate(urls, 1):
        try:
            response = requests.get(url, timeout=10)
            if response.status_code == 200:
                print(f"[OK] {url} - Status: {response.status_code}")
                accessible_urls.append(url)
            else:
                print(f"[ERROR] {url} - Status: {response.status_code}")
                inaccessible_urls.append(url)
        except Exception as e:
            print(f"[ERROR] {url} - Error: {str(e)}")
            inaccessible_urls.append(url)

        # Be respectful to the server
        time.sleep(0.5)

    print(f"\nSummary:")
    print(f"Accessible URLs: {len(accessible_urls)}")
    print(f"Inaccessible URLs: {len(inaccessible_urls)}")

    print(f"\nAccessible URLs:")
    for url in accessible_urls:
        print(f"  - {url}")

    return accessible_urls

if __name__ == "__main__":
    accessible_urls = test_urls()
    print(f"\nYou can use these {len(accessible_urls)} URLs for your RAG pipeline:")
    for url in accessible_urls:
        print(f"  '{url}',")