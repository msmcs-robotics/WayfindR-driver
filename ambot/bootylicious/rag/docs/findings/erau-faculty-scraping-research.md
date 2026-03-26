# ERAU Faculty Directory Scraping Research

Date: 2026-03-26

## Summary

ERAU has two publicly accessible (no login required) faculty directory systems:
1. **Department pages** on `daytonabeach.erau.edu` -- listing pages with expandable cards
2. **Faculty directory site** at `faculty.erau.edu` -- searchable directory with individual profile pages

The `faculty.erau.edu` site is the better scraping target. It has structured individual profile pages with rich data fields and predictable URL patterns.

---

## Key URLs

### EECS Department Faculty Listing
- **URL:** `https://faculty.erau.edu/departments/DCOEECSSE`
- **Pagination:** `?p=2` for page 2 (20 results per page)
- **Total EECS faculty:** 35
- **Department code:** `DCOEECSSE` (Daytona Beach College of Engineering, EECS)

### Individual Faculty Profile Pages
- **URL pattern:** `https://faculty.erau.edu/{FirstName}.{LastName}`
- **Examples:**
  - `https://faculty.erau.edu/Richard.Stansbury`
  - `https://faculty.erau.edu/Brian.Butka`
  - `https://faculty.erau.edu/Mustafa.Akbas`

### Other Entry Points
- Browse all faculty: `https://faculty.erau.edu/faculty`
- Browse by college: `https://faculty.erau.edu/colleges`
- Browse by department: `https://faculty.erau.edu/departments`
- EECS page on main site: `https://daytonabeach.erau.edu/college-engineering/electrical-engineering-computer-science/faculty`

### Photo URL Pattern
- `https://webforms.erau.edu/common/services/peoplesearch/img/{USERNAME}.jpg`
- Example: `https://webforms.erau.edu/common/services/peoplesearch/img/STANSBUR.jpg`

---

## Public Accessibility

All pages are **publicly accessible** with no login required:
- Department listing pages: YES
- Individual profile pages: YES
- Faculty photos: YES
- No authentication headers or cookies needed for basic GET requests

---

## Data Fields Available on Individual Profile Pages

From analysis of multiple profiles (Stansbury, Butka), these fields are available:

| Field | Available | Example |
|-------|-----------|---------|
| Full Name | YES | "Richard S. Stansbury" |
| Title/Position | YES | "Associate Professor and Director - ASSURE Center of Excellence" |
| Email | YES | "stansbur@erau.edu" |
| Department | YES | "Electrical Engineering and Computer Science Dept" |
| College | YES | "College of Engineering" |
| Campus | YES | "Daytona Beach Campus" |
| Photo URL | YES | webforms.erau.edu/common/services/peoplesearch/img/... |
| Education (degrees) | YES | List of degrees with institution names |
| Areas of Expertise | YES | Free text keywords |
| Currently Teaching | YES | Course codes and names |
| Courses Taught (past) | YES | Course codes |
| Research Focus / Bio | YES | Free text paragraph |
| Publications | SOME | Some profiles list 80+ publications; not all have them |
| Office Hours | SOME | Free text, not standardized |
| Office Location | RARE | Not consistently present |
| Phone Number | RARE | Not consistently present |
| Associated Lab | SOME | e.g., "EECS Capstone Design Laboratory" |

---

## Complete EECS Faculty List (35 members)

### Page 1 (1-20)
| # | Display Name | Profile URL Path |
|---|-------------|-----------------|
| 1 | Parham Ahmady Phoulady | /Hady.AhmadyPhoulady |
| 2 | M. Ilhan Akbas | /Mustafa.Akbas |
| 3 | Farahzad Behi | /Farahzad.Behi |
| 4 | David Bethelmy | /David.Bethelmy |
| 5 | Michael Bosser | /Michael.Bosser |
| 6 | Kimberly Brantley | /Kimberly.Brantley |
| 7 | Brian Butka | /Brian.Butka |
| 8 | Christopher Cerqueira | /Christopher.Cerqueira |
| 9 | Ilteris Demirkiran | /Ilteris.Demirkiran |
| 10 | Daniel Diessner | /Daniel.Diessner |
| 11 | Ke Feng | /Ke.Feng |
| 12 | Keith Garfield | /Keith.Garfield |
| 13 | Quentin Goss | /Quentin.Goss |
| 14 | Rogelio Gracia Otalvaro | /Rogelio.GraciaOtalvaro |
| 15 | Caleb Hall | /Caleb.Hall1 |
| 16 | Siyao Li | /Siyao.Li |
| 17 | Jianhua Liu | /Jianhua.Liu |
| 18 | Shafika Showkat Moni | /ShafikaShowkat.Moni |
| 19 | Vidhyashree Nagaraju | /Vidhyashree.Nagaraju |
| 20 | Laxima Niure Kandel | /Laxima.NiureKandel |

### Page 2 (21-35)
| # | Display Name | Profile URL Path |
|---|-------------|-----------------|
| 21 | Omar Ochoa | /Omar.Ochoa |
| 22 | Juan Ortiz Couder | /Juan.OrtizCouder |
| 23 | Berker Pekoz | /Berker.Pekoz |
| 24 | Daniel Penny | /Daniel.Penny |
| 25 | George Pozek | /George.Pozek |
| 26 | Sarah Reynolds | /Sarah.Reynolds1 |
| 27 | Eduardo Rojas | /Eduardo.Rojas |
| 28 | Davian Rosario-Ortiz | /Davian.RosarioOrtiz |
| 29 | Richard S. Stansbury | /Richard.Stansbury |
| 30 | Rumia Sultana | /Rumia.Sultana |
| 31 | Massood Towhidnejad | /Massood.Towhidnejad |
| 32 | Raul Alejandro Vargas Acosta | /Alejandro.Vargas |
| 33 | Bryan Watson | /Bryan.Watson3 |
| 34 | Fan Yang | /Fan.Yang1 |
| 35 | Thomas Yang | /Tianyu.Yang |

**NOTE:** Display names do not always match URL slugs (e.g., "M. Ilhan Akbas" -> `/Mustafa.Akbas`, "Thomas Yang" -> `/Tianyu.Yang`). The URL slug uses the faculty member's legal/system first name, not their display/preferred name. Some have numeric suffixes (e.g., `Hall1`, `Reynolds1`, `Watson3`, `Yang1`) to disambiguate common names.

---

## URL Pattern Quirks

- **Name format:** `FirstName.LastName` (CamelCase, no spaces)
- **Compound last names:** Spaces/hyphens removed (e.g., `GraciaOtalvaro`, `RosarioOrtiz`, `NiureKandel`, `OrtizCouder`)
- **Compound first names:** Concatenated (e.g., `ShafikaShowkat.Moni`)
- **Disambiguation:** Numeric suffix appended (e.g., `Caleb.Hall1`, `Fan.Yang1`)
- **Case:** Generally capitalized but not always consistent (one instance of `douglas.adams` seen in global directory)

---

## Scraping Strategy

### Recommended Approach

```
Step 1: Scrape department listing pages to get all profile URL slugs
        GET https://faculty.erau.edu/departments/DCOEECSSE
        GET https://faculty.erau.edu/departments/DCOEECSSE?p=2
        Parse <a href="/FirstName.LastName"> links

Step 2: For each profile URL, fetch the individual page
        GET https://faculty.erau.edu/{slug}
        Parse structured fields from HTML

Step 3: Store extracted data as JSON/CSV for RAG ingestion
```

### Python Libraries Needed
- `requests` -- HTTP fetching
- `beautifulsoup4` -- HTML parsing
- `concurrent.futures.ThreadPoolExecutor` -- parallel profile fetching (10 workers recommended)
- `pandas` (optional) -- CSV export
- `time.sleep` -- rate limiting (be polite, 0.5-1s between requests)

### Key Parsing Hints
- The department listing page uses standard HTML `<a>` tags with `href="/FirstName.LastName"` for profile links
- Profile pages have structured sections for Education, Expertise, Courses, etc.
- Publications may be in a separate section or expandable area
- Photo URLs follow pattern: `webforms.erau.edu/common/services/peoplesearch/img/{USERNAME}.jpg`
- Pagination uses `?p=N` parameter, 20 results per page

### Example Skeleton Code

```python
import requests
from bs4 import BeautifulSoup
from concurrent.futures import ThreadPoolExecutor
import time
import json

BASE_URL = "https://faculty.erau.edu"
DEPT_URL = f"{BASE_URL}/departments/DCOEECSSE"

def get_faculty_slugs():
    """Scrape department pages to get all faculty profile URL slugs."""
    slugs = []
    page = 1
    while True:
        url = DEPT_URL if page == 1 else f"{DEPT_URL}?p={page}"
        resp = requests.get(url)
        soup = BeautifulSoup(resp.text, "html.parser")
        # Find faculty profile links -- adjust selector to actual HTML structure
        links = soup.select("a[href^='/']")  # refine selector after inspecting HTML
        new_slugs = [a["href"] for a in links if "." in a["href"] and "/departments" not in a["href"]]
        if not new_slugs:
            break
        slugs.extend(new_slugs)
        page += 1
        time.sleep(0.5)
    return list(set(slugs))

def scrape_profile(slug):
    """Fetch and parse an individual faculty profile page."""
    url = f"{BASE_URL}{slug}"
    resp = requests.get(url)
    soup = BeautifulSoup(resp.text, "html.parser")
    # Extract fields -- selectors need tuning to actual HTML structure
    profile = {
        "url": url,
        "slug": slug,
        # Parse name, title, email, education, expertise, courses, bio, etc.
        # from soup using appropriate CSS selectors
    }
    return profile

def main():
    slugs = get_faculty_slugs()
    print(f"Found {len(slugs)} faculty profiles")

    profiles = []
    with ThreadPoolExecutor(max_workers=5) as executor:
        futures = {executor.submit(scrape_profile, s): s for s in slugs}
        for future in futures:
            time.sleep(0.3)  # rate limiting
            profiles.append(future.result())

    with open("erau_eecs_faculty.json", "w") as f:
        json.dump(profiles, f, indent=2)

if __name__ == "__main__":
    main()
```

**Note:** The CSS selectors in the skeleton code are placeholders. The actual HTML structure needs to be inspected to write precise selectors for each field. The first step of implementation should be to download one profile page, pretty-print the HTML, and identify the exact tags/classes used for each data field.

---

## Ethical Considerations

- Add a `User-Agent` header identifying the bot purpose
- Rate limit requests (0.5-1s delay between fetches)
- Check `robots.txt` at `https://faculty.erau.edu/robots.txt` before scraping
- This data is publicly available educational information
- Do not scrape or store student data, only faculty profiles
- Consider caching responses to avoid redundant requests during development

---

## References

- [UB Faculty Web Scraper (GitHub)](https://github.com/pChitral/University-at-Buffalo-Faculty-Web-Scraper) -- complete example of university faculty scraping with ThreadPoolExecutor
- [Web Scraping Faculty Data (Medium)](https://chitralpatil.medium.com/web-scraping-with-python-a-comprehensive-guide-to-extracting-faculty-data-and-boosting-performance-49bc9e2341b5) -- step-by-step guide with concurrent execution
- [Beautiful Soup Web Scraper (Real Python)](https://realpython.com/beautiful-soup-web-scraper-python/) -- general BeautifulSoup tutorial
