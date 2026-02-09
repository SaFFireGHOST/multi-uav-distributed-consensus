
import sys

def try_read_pdf(path):
    # Try pypdf
    try:
        import pypdf
        print("Using pypdf", file=sys.stderr)
        reader = pypdf.PdfReader(path)
        text = ""
        for page in reader.pages:
            text += page.extract_text() + "\n"
        return text
    except ImportError:
        pass
    except Exception as e:
        print(f"pypdf failed: {e}", file=sys.stderr)

    # Try PyPDF2
    try:
        import PyPDF2
        print("Using PyPDF2", file=sys.stderr)
        reader = PyPDF2.PdfReader(path)
        text = ""
        for page in reader.pages:
            text += page.extract_text() + "\n"
        return text
    except ImportError:
        pass
    except Exception as e:
        print(f"PyPDF2 failed: {e}", file=sys.stderr)

    # Try pdfminer.six
    try:
        from pdfminer.high_level import extract_text
        print("Using pdfminer.six", file=sys.stderr)
        return extract_text(path)
    except ImportError:
        pass
    except Exception as e:
        print(f"pdfminer failed: {e}", file=sys.stderr)

    return None

if __name__ == "__main__":
    pdf_path = "drones-06-00226-v2 (1).pdf"
    text = try_read_pdf(pdf_path)
    if text:
        with open("paper_content.txt", "w", encoding="utf-8") as f:
            f.write(text)
        print("Successfully wrote to paper_content.txt")
    else:
        print("FAILED_TO_read_PDF_NO_LIBRARY")
