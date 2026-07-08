import os
import re
import bz2
import xml.etree.ElementTree as ET
import torch
from torch.utils.data import IterableDataset, DataLoader
from transformers import GPT2Config, GPT2LMHeadModel, AutoTokenizer

# ==========================================
# 1. ON-THE-FLY WIKIPEDIA DUMP STREAMER
# ==========================================

class WikiXmlStreamer:
    """
    Strips raw MediaWiki XML and returns cleaned plain-text articles
    on the fly with virtually zero RAM footprint.
    """
    def __init__(self, file_path):
        self.file_path = file_path
        # Basic regex to strip templates, internal links, and MediaWiki syntax
        self.cleanup_patterns = [
            (re.compile(r'\{\{[^}]*\}\}'), ''),                 # Strip templates
            (re.compile(r'\[\[(?:[^|\]]*\|)?([^\]]+)\]\]'), r'\1'), # Simplify links [[A|B]] -> B
            (re.compile(r'==+\s*([^=]+)\s*==+'), r'\1'),         # Clean headers === Title === -> Title
            (re.compile(r'\'\'\'?'), ''),                       # Strip bold/italics markers
            (re.compile(r'&[a-zA-Z0-9#]+;'), ' '),              # Strip HTML entities
            (re.compile(r'\n{3,}'), '\n\n')                      # Normalize spacing
        ]

    def clean_text(self, text):
        if not text:
            return ""
        # Quick skip for redirects or non-content namespaces
        if text.strip().startswith("#REDIRECT") or text.strip().startswith("#redirect"):
            return ""
        for pattern, replacement in self.cleanup_patterns:
            text = pattern.sub(replacement, text)
        return text.strip()

    def __iter__(self):
        # Read the compressed bz2 stream on the fly
        with bz2.BZ2File(self.file_path, 'rb') as f:
            # Parse XML incrementally node-by-node (saves RAM)
            context = ET.iterparse(f, events=('end',))
            # Find the namespace dynamically
            ns = ""
            for event, elem in context:
                if elem.tag.endswith('page'):
                    # Capture namespace from the first element
                    if not ns and '}' in elem.tag:
                        ns = elem.tag.split('}')[0] + '}'
                    
                    title_elem = elem.find(f'{ns}title')
                    ns_elem = elem.find(f'{ns}ns')
                    revision = elem.find(f'{ns}revision')
                    
                    # Ensure it is a main namespace article (ns == 0) and not a helper page
                    if ns_elem is not None and ns_elem.text == '0' and revision is not None:
                        text_elem = revision.find(f'{ns}text')
                        if text_elem is not None and text_elem.text:
                            cleaned = self.clean_text(text_elem.text)
                            if cleaned and len(cleaned) > 200: # Filter short stubs
                                yield f"{title_elem.text}\n\n{cleaned}"
                                
                    # Clear out parent nodes from memory to prevent memory leaks
                    elem.clear()

# ==========================================
# 2. ITERABLE PYTORCH DATASET WITH TOKENIZER
# ==========================================

class TokenizedWikiDataset(IterableDataset):
    def __init__(self, xml_path, tokenizer_name="gpt2", max_length=1024):
        super().__init__()
        self.streamer = WikiXmlStreamer(xml_path)
        self.tokenizer = AutoTokenizer.from_pretrained(tokenizer_name)
        if self.tokenizer.pad_token is None:
            self.tokenizer.pad_token = self.tokenizer.eos_token
        self.max_length = max_length

    def __iter__(self):
        for text in self.streamer:
            tokens = self.tokenizer(
                text, 
                truncation=True, 
                max_length=self.max_length, 
                return_tensors="pt"
            )
            input_ids = tokens["input_ids"].squeeze(0)
            
            # Label is simply shifted input_ids. Trainer/PyTorch will handle loss offset automatically.
            yield {
                "input_ids": input_ids,
                "labels": input_ids.clone()
            }

def collate_fn(batch):
    # Pad sequences dynamically to match the longest item in the batch
    input_ids = [item["input_ids"] for item in batch]
    labels = [item["labels"] for item in batch]
    
    padded_inputs = torch.nn.utils.rnn.pad_sequence(input_ids, batch_first=True, padding_value=50256)
    padded_labels = torch.nn.utils.rnn.pad_sequence(labels, batch_first=True, padding_value=-100) # -100 ignores loss on padding
    
    return {
        "input_ids": padded_inputs,
        "labels": padded_labels
    }

# ==========================================
# 3. CORE TRAINING PIPELINE (NATIVE XPU)
# ==========================================

def train():
    # List of possible locations for your Wikipedia dump file
    possible_paths = [
        "enwiki-20250801-pages-articles-multistream.xml.bz2",
        "/run/media/isaac/Main/enwiki-20250801-pages-articles-multistream.xml.bz2"
    ]
    
    dump_path = None
    for path in possible_paths:
        if os.path.exists(path):
            dump_path = path
            break
            
    if not dump_path:
        raise FileNotFoundError(
            f"Could not find local Wikipedia dump file at any of the searched paths: {possible_paths}"
        )

    print(f"Initializing streaming dataset using dump at: {dump_path}")
    dataset = TokenizedWikiDataset(dump_path)
    
    # Custom DataLoader directly handling raw multi-stream iteration
    dataloader = DataLoader(dataset, batch_size=8, collate_fn=collate_fn, num_workers=2)

    # Define 100M-parameter model (GPT-2 medium-ish scale config)
    print("Initializing 100M Model architecture...")
    config = GPT2Config(
        vocab_size=50257,
        n_positions=1024,
        n_embd=768,      # Hidden size
        n_layer=12,      # Transformer layers
        n_head=12,       # Attention heads
        bos_token_id=50256,
        eos_token_id=50256
    )
    model = GPT2LMHeadModel(config)
    print(f"Total model parameters: {model.num_parameters():,}")

    # Set up native XPU device
    device = torch.device("xpu")
    print(f"Targeting Native device: {device}")
    model = model.to(device)

    # Optional: Upstream PyTorch torch.compile works seamlessly with the Intel XPU backend
    # This replaces the need for old 'ipex.optimize()' calls and offers better kernel fusion
    try:
        print("Compiling model natively for Intel XPU graph optimizations...")
        model = torch.compile(model)
    except Exception as e:
        print(f"Native torch.compile omitted or failed (falling back to Eager XPU mode): {e}")

    # Standard PyTorch Optimizer
    optimizer = torch.optim.AdamW(model.parameters(), lr=6e-4, weight_decay=0.1)

    model.train()
    grad_accum_steps = 4
    step = 0
    running_loss = 0.0

    print("Beginning Training Loop. Streaming text directly from compressed .bz2 dump...")
    
    for epoch in range(1):
        optimizer.zero_grad()
        for batch_idx, batch in enumerate(dataloader):
            input_ids = batch["input_ids"].to(device)
            labels = batch["labels"].to(device)

            # Native PyTorch Autocast with XPU device type for Bfloat16 support (ideal for Xe-cores)
            with torch.amp.autocast(device_type="xpu", enabled=True, dtype=torch.bfloat16):
                outputs = model(input_ids=input_ids, labels=labels)
                loss = outputs.loss / grad_accum_steps

            loss.backward()
            running_loss += loss.item() * grad_accum_steps

            if (batch_idx + 1) % grad_accum_steps == 0:
                optimizer.step()
                optimizer.zero_grad()
                step += 1

                if step % 10 == 0:
                    print(f"Step: {step} | Batch: {batch_idx+1} | Loss: {running_loss / 10:.4f}")
                    running_loss = 0.0

                # Save checkpoint every 5,000 steps
                if step % 5000 == 0:
                    checkpoint_dir = f"./checkpoint-{step}"
                    os.makedirs(checkpoint_dir, exist_ok=True)
                    
                    # Uncompile model if saving natively
                    uncompiled_model = model._orig_mod if hasattr(model, "_orig_mod") else model
                    uncompiled_model.save_pretrained(checkpoint_dir)
                    print(f"Checkpoint saved to {checkpoint_dir}")

if __name__ == "__main__":
    train()
